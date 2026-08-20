#pragma once
#include <algorithm>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

class IHexapodAPI;

typedef struct Pose {
        double x;
        double y;
        double z;
        double rx;
        double ry;
        double rz;
};

enum class SensorMode { DISABLED = 0, ENABLED = 1, POWERSAVE = 2 };

enum class FrefMethod {
    DEFAULT = 0,
    SEQUENTIAL = 1,
    ZSAFE = 2,
    XYSAFE = 3,
};

enum class FrefDirection {
    DEFAULT = 0,
    POSITIVE = 256,  // 0x0100h
    NEGATIVE = 512,  // 0x0200h
    REVERSE = 4096,  // 0x1000h
};

enum class PivotMode { RELATIVE = 0, FIXED = 1 };

enum class Axis {
    X = 1,
    Y = 2,
    Z = 0,
};

enum class MoveStatus {
    STOPPED = 0,
    HOLDING = 1,
    MOVING = 2,
    CALIBRATING = 3,
    REFERENCING = 4,
    STANDBY = 5,
};

enum class StatusCode {
    OK = 0,           // The function call was successful.
    OTHER_ERROR = 1,  // An uncategorized error has occurred.
    SYSTEM_NOT_INITIALIZED_ERROR =
        2,  // This error is returned if the initialization of the MCS controller fails in the call
            // to Smarpod_InitSystems or if Smarpod_Initialize or Smarpod_ReleaseAll are called
            // without an initialized MCS controller.
    NO_SYSTEMS_FOUND_ERROR =
        3,  // Returned by Smarpod_InitSystems if no MCS controller systems can be found.
    INVALID_PARAMETER_ERROR = 4,  // Returned by various functions if a parameter value is invalid
                                  // and there is no other error codethat is more specific.
    COMMUNICATION_ERROR =
        5,  // Can be returned by various functions if there is a communication problem with the
            // MCScontroller. See 3.3 “Communication Failures” for more information.
    UNKNOWN_PROPERTY_ERROR =
        6,  // Returned by property related function if an unknown property is passed.
    RESOURCE_TOO_OLD_ERROR = 7,  // Returned if a recource the SmarPod (library) depends on is too
                                 // old (e.g. the MCSControl library).
    FEATURE_UNAVAILABLE_ERROR = 8,  // Returned if a certain feature is not supported by the soft-
                                    // or hardware, e.g. acceleration-control.
    INVALID_SYSTEM_LOCATOR_ERROR =
        9,  // Returned by Smarpod_Open if the locator string has a wrong formatting.
    QUERYBUFFER_SIZE_ERROR = 10,  // Returned by functions that write data in a binary or char
                                  // buffer (e.g. Smarpod_FindSystems) ifthe user-supplied buffer is
                                  // too small to hold the returned data.
    COMMUNICATION_TIMEOUT_ERROR =
        11,             // Returned if an expected answer from the MCS controller takes too long.
    DRIVER_ERROR = 12,  // Returned if a driver that is needed for communicating with the controller
                        // could not be loaded.
    STATUS_CODE_UNKNOWN_ERROR = 500,  // Returned by function Smarpod_GetStatusInfo if it is called
                                      // with an unknown status code.
    INVALID_ID_ERROR = 501,  // Returned by several functions if the smarpodId parameter is not a
                             // valid ID that was returnedby a successful call to Smarpod_Open.
    HARDWARE_MODEL_UNKNOWN_ERROR = 503,  // Returned by the SmarPod initialization if called with an
                                         // unknown hardware-model code.
    WRONG_COMM_MODE_ERROR = 504,         // Returned by SmarPod initialization if the controller was
                                         // initialized in synchronous mode.
    NOT_INITIALIZED_ERROR = 505,         // Returned by various functions if no SmarPod has been
                                         // initialized for the given SmarPod ID.
    INVALID_SYSTEM_ID_ERROR =
        506,  // Returned by Smarpod_InitSystems and Smarpod_Initialize if an MCS System ID couldnot
              // be found in the list of connected MCS devices.
    NOT_ENOUGH_CHANNELS_ERROR =
        507,  // Returned by Smarpod_Initialize if the controller specified by the System ID has
              // lesschannels than required for the SmarPod model.
    SENSORS_DISABLED_ERROR =
        510,  // Returned by commands that move the SmarPod if the sensor mode is disabled.
    WRONG_SENSOR_TYPE_ERROR =
        511,  // No longer used. Replaced by SMARPOD_SYSTEM_CONFIGURATION_ERROR.
    SYSTEM_CONFIGURATION_ERROR =
        512,  // Returned by function Smarpod_Initialize if the internal MCS configuration does not
              // matchthe SmarPod model passed to the initialization function. See 3.1 “System
              // Configuration Error”and Smarpod_ConfigureSystem.
    SENSOR_NOT_FOUND_ERROR =
        513,  // Returned by function Smarpod_Initialize if one or more sensors of the
              // SmarPodpositioners could not be detected. This error can indicate a not connected
              // positioner or acommunication problem with a positioner.
    STOPPED_ERROR = 514,  // Returned by Smarpod_Calibrate and Smarpod_FindReferenceMarks if
                          // Smarpod_Stop iscalled while they are executing. Returned by
                          // Smarpod_Move if called with waitForCompletion = 1.
    BUSY_ERROR = 515,     // Returned by functions Smarpod_Calibrate, Smarpod_FindReferenceMarks,
                          // Smarpod_Moveand Smarpod_StopAndHold if the SmarPod is busy and the
                          // function cannot be executed. E.g.when the SmarPod is referencing or
                       // calibrating, a call of Smarpod_Move would return withSMARPOD_BUSY_ERROR.
                       // Smarpod_Move will also returns with SMARPOD_BUSY_ERROR if anothermove
                       // command that has been called with waitForCompletion=1 is still executing.
    NOT_REFERENCED_ERROR =
        550,  // Returned by Smarpod_Move if the reference marks of the positioners are not known.
              // See section 2.4.5 “Finding Reference Marks“.
    POSE_UNREACHABLE_ERROR = 551,  // Returned by Smarpod_Move if the pose cannot be reached.
    COMMAND_OVERRIDDEN_ERROR =
        552,  // When the software commands a movement which is then interrupted by the HandControl
              // Module, an error of this type is generated.
    ENDSTOP_REACHED_ERROR = 553,  // This error is returned if the target pose could not be reached
                                  // because a mechanical endstop was detected.
    NOT_STOPPED_ERROR = 554,  // Returned if a command is called which requires that the SmarPod is
                              // stopped but it ismoving or holding.
    COULD_NOT_REFERENCE_ERROR = 555,  // This error is returned by Smarpod_FindReferenceMarks if the
                                      // reference marks could notbe found.
    COULD_NOT_CALIBRATE_ERROR = 556,  // This error is returned by Smarpod_Calibrate if the
                                      // calibration of the positioner sensorshas failed.
};

class Hexapod {
    public:
        Hexapod(unsigned int id) : id(id) {}
        virtual ~Hexapod() = default;

        virtual void SetMaxFrequency(unsigned int frequency) = 0;
        virtual unsigned int GetMaxFrequency() = 0;
        virtual void SetSpeed(double speed, bool enableSpeedControl) = 0;
        virtual std::tuple<double, bool> GetSpeed() = 0;
        virtual void SetAcceleration(double acceleration, bool enableAccelControl) = 0;
        virtual std::tuple<double, bool> GetAcceleration() = 0;
        virtual void FindReferenceMarks() = 0;
        virtual void Calibrate() = 0;
        virtual bool IsReferenced() = 0;
        virtual void SetSensorMode(SensorMode mode) = 0;
        virtual SensorMode GetSensorMode() = 0;
        virtual void SetPivot(double x, double y, double z) = 0;
        virtual std::tuple<double, double, double> GetPivot() = 0;
        virtual bool IsPoseReachable(const Pose& pose) = 0;
        virtual Pose GetPose() = 0;
        virtual void Move(const Pose& pose, unsigned int holdTime, bool waitForCompletion) = 0;
        virtual void Stop() = 0;
        virtual void StopAndHold(unsigned int holdTime) = 0;
        virtual void Standby() = 0;
        virtual void SetCoordinateSystem(const Pose& pose) = 0;
        virtual Pose GetCoordinateSystem() = 0;
        virtual void SetCurrentPoseAsZero() = 0;
        virtual void SetAxesOrientation(double rx, double ry, double rz) = 0;
        virtual std::tuple<double, double, double> GetAxesOrientation() = 0;
        virtual std::string GetLocator() = 0;
        virtual void ConfigureSystem() = 0;
        virtual void SetFindRefDirection(Axis ax, FrefDirection direction) = 0;
        virtual FrefDirection GetFindRefDirection(Axis ax) = 0;
        virtual void SetPivotMode(PivotMode mode) = 0;
        virtual PivotMode GetPivotMode() = 0;
        virtual void SetFindRefMethod(FrefMethod frefMethod) = 0;
        virtual FrefMethod GetFindRefMethod() = 0;
        virtual void SetFindRefAndCalibFreq(double frefAndCalibFreq) = 0;
        virtual double GetFindRefAndCalibFreq() = 0;
        virtual MoveStatus GetMoveStatus() = 0;

        unsigned int GetID() { return this->id; }
        IHexapodAPI* GetAPI() { return this->api; }

    private:
        unsigned int id;

        template <typename T>
        void SetProperty(unsigned int propertyID, T value);

        template <typename T>
        T GetProperty(unsigned int propertyID);

        IHexapodAPI* api = nullptr;
};

class IHexapodAPI {
    public:
        virtual std::tuple<int, int, int> GetVersion() = 0;
        virtual std::vector<unsigned int> GetSupportedModels() = 0;
        virtual std::string GetModelName(unsigned int model) = 0;
        virtual std::string GetStatusMessage(StatusCode status) = 0;
        virtual std::unique_ptr<Hexapod> Open(unsigned int model, std::string locator) = 0;
        virtual void Close(const std::unique_ptr<Hexapod>& hexapod) = 0;
        virtual std::vector<std::string> FindSystems() = 0;
        virtual void ConfigureController(unsigned int model, std::string locator) = 0;

        std::string GetVersionStr() {
            auto [major, minor, update] = this->GetVersion();
            return std::to_string(major) + "." + std::to_string(minor) + "." +
                   std::to_string(update);
        }

        bool AreModelAndLocatorValid(unsigned int model, std::string locator) {
            return IsModelSupported(model) && IsLocatorFound(locator);
        }

    protected:
        bool IsModelSupported(unsigned int model) {
            std::vector<unsigned int> models = GetSupportedModels();
            if (std::find(models.begin(), models.end(), model) != models.end()) return true;
            return false;
        }

        bool IsLocatorFound(std::string locator) {
            std::vector<std::string> locators = FindSystems();
            if (std::find(locators.begin(), locators.end(), locator) != locators.end()) return true;
            return false;
        }

    private:
        std::vector<Hexapod> hexapods;

};  // HexapodAPI