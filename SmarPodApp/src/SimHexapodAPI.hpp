#pragma once
#include <atomic>
#include <chrono>

#include "IHexapodAPI.hpp"

class SimHexapod : public Hexapod {
    public:
        SimHexapod(unsigned int id) : Hexapod(id) {}

        virtual void SetMaxFrequency(unsigned int frequency);
        virtual unsigned int GetMaxFrequency();
        virtual void SetSpeed(double speed, bool enableSpeedControl);
        virtual std::tuple<double, bool> GetSpeed();
        virtual void SetAcceleration(double acceleration, bool enableAccelControl);
        virtual std::tuple<double, bool> GetAcceleration();
        virtual void FindReferenceMarks();
        virtual void Calibrate();
        virtual bool IsReferenced();
        virtual void SetSensorMode(SensorMode mode);
        virtual SensorMode GetSensorMode();
        virtual void SetPivot(double x, double y, double z);
        virtual std::tuple<double, double, double> GetPivot();
        virtual bool IsPoseReachable(const Pose& pose);
        virtual Pose GetPose();
        virtual void Move(const Pose& pose, unsigned int holdTime, bool waitForCompletion);
        virtual void Stop();
        virtual void StopAndHold(unsigned int holdTime);
        virtual void Standby();
        virtual void SetCoordinateSystem(const Pose& pose);
        virtual Pose GetCoordinateSystem();
        virtual void SetCurrentPoseAsZero();
        virtual void SetAxesOrientation(double rx, double ry, double rz);
        virtual std::tuple<double, double, double> GetAxesOrientation();
        virtual std::string GetLocator();
        virtual void ConfigureSystem();
        virtual void SetFindRefDirection(Axis ax, FrefDirection direction);
        virtual FrefDirection GetFindRefDirection(Axis ax);
        virtual void SetPivotMode(PivotMode mode);
        virtual PivotMode GetPivotMode();
        virtual void SetFindRefMethod(FrefMethod frefMethod);
        virtual FrefMethod GetFindRefMethod();
        virtual void SetFindRefAndCalibFreq(double frefAndCalibFreq);
        virtual double GetFindRefAndCalibFreq();
        virtual MoveStatus GetMoveStatus();

    private:
        bool referenced = false, calibrated = false;
        Pose currentPose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        SensorMode sensorMode = SensorMode::POWERSAVE;
        PivotMode pivotMode = PivotMode::RELATIVE;
        FrefDirection xFrefDir = FrefDirection::DEFAULT, yFrefDir = FrefDirection::DEFAULT,
                      zFrefDir = FrefDirection::DEFAULT;
        FrefMethod frefMethod = FrefMethod::DEFAULT;
        std::atomic<MoveStatus> moveStatus{MoveStatus::STOPPED};
        double speed = 0.001, acceleration = 0.01;  // 1 mm/s, 10 mm/s^2
        bool speedControlEnabled = true, acclControlEnabled = false;
        Pose coordSystem = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        std::string locator = "sim:123456";
        unsigned int maxFrequency = 18500;  // Hz, typical SmarPod default
        double frefAndCalibFreq = 8000;
        double pivotX = 0, pivotY = 0, pivotZ = 0;
        double axisRx = 0, axisRy = 0, axisRz = 0;

        // Time-based motion model
        std::chrono::steady_clock::time_point moveStartTime;
        std::chrono::steady_clock::time_point moveEndTime;
        std::chrono::steady_clock::time_point holdEndTime;
        bool holdInfinite = false;
        Pose moveStartPose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        Pose moveTargetPose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        bool isWithinWorkspace(const Pose& pose) const;
        double computeMoveDuration(const Pose& from, const Pose& to) const;
        void advance();
        void jogAllAxes(MoveStatus activeStatus);
};

class SimHexapodAPI : public IHexapodAPI {
    public:
        SimHexapodAPI(const char* locator) : locator(locator) {}
        std::tuple<int, int, int> GetVersion() override;
        std::vector<unsigned int> GetSupportedModels() override;
        std::string GetModelName(unsigned int model) override;
        std::string GetStatusMessage(StatusCode status) override;
        std::unique_ptr<Hexapod> Open(unsigned int model, std::string locator) override;
        void Close(const std::unique_ptr<Hexapod>& hexapod) override;
        std::vector<std::string> FindSystems() override;
        void ConfigureController(unsigned int model, std::string locator) override;

    private:
        unsigned int nextId = 0;
        std::string locator;
};
