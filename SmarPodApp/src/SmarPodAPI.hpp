#pragma once
#include <stdexcept>

#include "IHexapodAPI.hpp"
#include "SmarPod.h"

std::string smarpodStatusMessage(StatusCode status);

inline void throwIfNotOk(StatusCode status, const char* func) {
    if (status != StatusCode::OK)
        throw std::runtime_error(std::string("Error when calling ") + func + ": " +
                                 smarpodStatusMessage(status));
}

#define THROW_IF_NOT_OK(status) throwIfNotOk(static_cast<StatusCode>(status), __func__)

enum class PropertyID {
    FREF_METHOD = 1000,
    FREF_XDIR = 1003,
    FREF_YDIR = 1004,
    FREF_ZDIR = 1002,
    FREF_AND_CAL_FREQ = 1020,
    PIVOT_MODE = 1010,
    POS_MIN_SPEED = 1100
};

class SmarPodHexapod : public Hexapod {
    public:
        SmarPodHexapod(unsigned int id) : Hexapod(id) {}

        void SetMaxFrequency(unsigned int frequency);
        unsigned int GetMaxFrequency();
        void SetSpeed(double speed, bool enableSpeedControl);
        std::tuple<double, bool> GetSpeed();
        void SetAcceleration(double acceleration, bool enableAccelControl);
        std::tuple<double, bool> GetAcceleration();

        void FindReferenceMarks();
        void Calibrate();
        bool IsReferenced();

        void SetSensorMode(SensorMode mode);
        SensorMode GetSensorMode();

        void SetPivot(double x, double y, double z);

        std::tuple<double, double, double> GetPivot();

        bool IsPoseReachable(const Pose& pose);

        Pose GetPose();

        void Move(const Pose& pose, unsigned int holdTime, bool waitForCompletion);

        void Stop();

        void StopAndHold(unsigned int holdTime);
        void Standby();
        void SetCoordinateSystem(const Pose& pose);
        Pose GetCoordinateSystem();

        void SetCurrentPoseAsZero();
        void SetAxesOrientation(double rx, double ry, double rz);
        std::tuple<double, double, double> GetAxesOrientation();

        std::string GetLocator();

        void ConfigureSystem();

        void SetFindRefDirection(Axis ax, FrefDirection direction);
        FrefDirection GetFindRefDirection(Axis ax);

        void SetPivotMode(PivotMode mode);
        PivotMode GetPivotMode();

        void SetFindRefMethod(FrefMethod);
        FrefMethod GetFindRefMethod();

        void SetFindRefAndCalibFreq(double frefAndCalibFreq);
        double GetFindRefAndCalibFreq();

        MoveStatus GetMoveStatus();

    private:
};

class SmarPodAPI : public IHexapodAPI {
    public:
        std::tuple<int, int, int> GetVersion() override;
        std::vector<unsigned int> GetSupportedModels() override;
        std::string GetModelName(unsigned int model) override;
        std::string GetStatusMessage(StatusCode status) override;
        std::unique_ptr<Hexapod> Open(unsigned int model, std::string locator) override;
        void Close(const std::unique_ptr<Hexapod>& hexapod) override;
        std::vector<std::string> FindSystems() override;
        void ConfigureController(unsigned int model, std::string locator) override;
};