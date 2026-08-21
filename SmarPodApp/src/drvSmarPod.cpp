/**
 * Main source file for the SmarPod EPICS driver
 *
 * Author: Afroza Haque
 *
 * Copyright (c) : Brookhaven National Laboratory, 2026
 *
 */

// Standard includes
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <algorithm>
#include <cmath>

// EPICS includes
#include <epicsExit.h>
#include <epicsExport.h>
#include <epicsStdio.h>
#include <epicsString.h>
#include <epicsThread.h>
#include <epicsTime.h>
#include <iocsh.h>
#include <spdlog/details/null_mutex.h>
#include <spdlog/sinks/base_sink.h>

#include "SmarPodStoredPose.hpp"
#include "drvSmarPod.hpp"

/**
 * @brief spdlog sink that mirrors each log message into the StatusMessage PV.
 *
 * Uses a null_mutex base so the only lock taken is the driver's asyn port lock,
 * avoiding a lock-ordering deadlock with threads that log while holding it.
 */
class StatusMessageSink : public spdlog::sinks::base_sink<spdlog::details::null_mutex> {
    public:
        explicit StatusMessageSink(SmarPod* driver) : driver(driver) {}

    protected:
        void sink_it_(const spdlog::details::log_msg& msg) override {
            this->driver->setStatusMessage(std::string(msg.payload.data(), msg.payload.size()));
        }
        void flush_() override {}

    private:
        SmarPod* driver;
};

/**
 * @brief External configuration function for SmarPod.
 *
 * Envokes the constructor to create a new SmarPod object
 * This is the function that initializes the driver, and is called in the IOC startup script
 *
 * NOTE: When implementing a new driver with PortDriverTemplate, your device may require additional
 * inputs from the user for connection (ex: IP address, serial number, etc.). These should be added
 * as additional IOC shell arguments for this function.
 *
 * @param portName Asyn port name for the SmarPod object instance.

 * @return asynSuccess
 */
extern "C" int SmarPodConfig(const char* portName, const char* locator, int modelNumber) {
    new SmarPod(portName, locator, modelNumber);

    return (asynSuccess);
}

/**
 * @brief Function for updating the log level of the SmarPod driver.
 *
 * This function is called from the IOC shell to set the log level of the driver.
 * @param logLevel The log level to set. Must be one of the following:
 * 0: trace
 * 1: debug
 * 2: info
 * 3: warn
 * 4: error
 * 5: critical
 * 6: off
 * @return asynSuccess if the log level was set successfully, asynError otherwise.
 */
extern "C" int SmarPodSetLogLevel(int logLevel) {
    spdlog::set_level(static_cast<spdlog::level::level_enum>(logLevel));
    return (asynSuccess);
}

extern "C" void referenceThread(void* pPvt) {
    SmarPod* pSmarPod = (SmarPod*) pPvt;
    pSmarPod->reference();
}

extern "C" void calibrateThread(void* pPvt) {
    SmarPod* pSmarPod = (SmarPod*) pPvt;
    pSmarPod->calibrate();
}

/**
 * @brief Callback function called when IOC is terminated.
 *
 * @param pPvt Pointer to SmarPod object
 */
static void exitCallbackC(void* pPvt) {
    SmarPod* pSmarPod = (SmarPod*) pPvt;
    delete pSmarPod;
}

static bool isLocatorSim(const char* locator) {
    return (locator == nullptr || strncmp(locator, "sim:", 4) == 0);
}

void SmarPod::reference() {
    try {
        this->pHexapod->FindReferenceMarks();
        this->updatePoseAndStatus();
        setIntegerParam(SmarPod_IsReferenced, static_cast<int>(this->pHexapod->IsReferenced()));
        callParamCallbacks();
    } catch (const std::exception& e) {
        spdlog::error("Error during reference: {}", e.what());
        setIntegerParam(SmarPod_IsReferenced, 0);
        callParamCallbacks();
    }
}

void SmarPod::calibrate() {
    try {
        this->pHexapod->Calibrate();
        this->updatePoseAndStatus();
    } catch (const std::exception& e) {
        spdlog::error("Error during calibration: {}", e.what());
    }
}

void SmarPod::spawnMoveThread(void (*threadFunc)(void*), const char* threadName) {
    epicsThreadOpts opts;
    opts.joinable = true;
    opts.stackSize = epicsThreadGetStackSize(epicsThreadStackMedium);
    opts.priority = epicsThreadPriorityMedium;
    this->moveThreadId =
        epicsThreadCreateOpt(threadName, (EPICSTHREADFUNC) threadFunc, this, &opts);
}

/**
 * @brief Handles write events to integer parameters
 *
 * @param pasynUser Pointer to asynUser for SmarPod instance
 * @param value Value written to the integer parameter
 * @return asynSuccess if write was successful, asynError otherwise
 */
asynStatus SmarPod::writeInt32(asynUser* pasynUser, epicsInt32 value) {
    int function = pasynUser->reason;

    asynStatus status = asynSuccess;

    const char* paramName;
    getParamName(function, &paramName);

    try {
        if (function == SmarPod_FindReferenceMarks && value) {
            this->spawnMoveThread(referenceThread, "SmarPodReferenceThread");
        } else if (function == SmarPod_Calibrate && value) {
            this->spawnMoveThread(calibrateThread, "SmarPodCalibrateThread");
        } else if (function == SmarPod_Move && value) {
            this->spawnMoveThread(moveThread, "SmarPodMoveThread");
        } else if (function == SmarPod_Stop && value) {
            int holdTime;
            getIntegerParam(SmarPod_HoldTime, &holdTime);
            if (holdTime <= 0) {
                this->pHexapod->Stop();
            } else {
                this->pHexapod->StopAndHold(static_cast<unsigned int>(holdTime));
            }
            if (this->moveThreadId) {
                epicsThreadMustJoin(this->moveThreadId);
                this->moveThreadId = nullptr;
            }
        } else if (function == SmarPod_Standby && value) {
            this->pHexapod->Standby();
            if (this->moveThreadId != nullptr) {
                epicsThreadMustJoin(this->moveThreadId);
                this->moveThreadId = nullptr;
            }
        } else if (function == SmarPod_SetPivot && value) {
            double x, y, z;
            getDoubleParam(SmarPod_PivotX, &x);
            getDoubleParam(SmarPod_PivotY, &y);
            getDoubleParam(SmarPod_PivotZ, &z);
            this->pHexapod->SetPivot(x, y, z);
            auto [px, py, pz] = this->pHexapod->GetPivot();
            setDoubleParam(SmarPod_PivotX, px);
            setDoubleParam(SmarPod_PivotY, py);
            setDoubleParam(SmarPod_PivotZ, pz);
            updatePoseAndStatus();
            spdlog::info("Set pivot to ({}, {}, {})", px, py, pz);
        } else if (function == SmarPod_SetCoordSys && value) {
            Pose csys;
            getDoubleParam(SmarPod_CoordSysX, &csys.x);
            getDoubleParam(SmarPod_CoordSysY, &csys.y);
            getDoubleParam(SmarPod_CoordSysZ, &csys.z);
            getDoubleParam(SmarPod_CoordSysRx, &csys.rx);
            getDoubleParam(SmarPod_CoordSysRy, &csys.ry);
            getDoubleParam(SmarPod_CoordSysRz, &csys.rz);
            this->pHexapod->SetCoordinateSystem(csys);
            Pose newCsys = this->pHexapod->GetCoordinateSystem();
            setDoubleParam(SmarPod_CoordSysX, newCsys.x);
            setDoubleParam(SmarPod_CoordSysY, newCsys.y);
            setDoubleParam(SmarPod_CoordSysZ, newCsys.z);
            setDoubleParam(SmarPod_CoordSysRx, newCsys.rx);
            setDoubleParam(SmarPod_CoordSysRy, newCsys.ry);
            setDoubleParam(SmarPod_CoordSysRz, newCsys.rz);
            updatePoseAndStatus();
            spdlog::info("Set coordinate system to ({}, {}, {}, {}, {}, {})", newCsys.x, newCsys.y,
                         newCsys.z, newCsys.rx, newCsys.ry, newCsys.rz);
        } else if (function == SmarPod_SetCurrentPoseAsZero && value) {
            this->pHexapod->SetCurrentPoseAsZero();
        } else if (function == SmarPod_SetAxesOrientation && value) {
            double rx, ry, rz;
            getDoubleParam(SmarPod_AxesRx, &rx);
            getDoubleParam(SmarPod_AxesRy, &ry);
            getDoubleParam(SmarPod_AxesRz, &rz);
            this->pHexapod->SetAxesOrientation(rx, ry, rz);
            auto [newRx, newRy, newRz] = this->pHexapod->GetAxesOrientation();
            setDoubleParam(SmarPod_AxesRx, newRx);
            setDoubleParam(SmarPod_AxesRy, newRy);
            setDoubleParam(SmarPod_AxesRz, newRz);
            updatePoseAndStatus();
            spdlog::info("Set axes orientation to ({}, {}, {})", newRx, newRy, newRz);
        } else if (function == SmarPod_MaxFrequency) {
            this->pHexapod->SetMaxFrequency(static_cast<unsigned int>(value));
            int maxFreq = static_cast<int>(this->pHexapod->GetMaxFrequency());
            setIntegerParam(SmarPod_MaxFrequency, maxFreq);
            spdlog::info("Set max frequency to {}", maxFreq);
        } else if (function == SmarPod_SensorMode) {
            this->pHexapod->SetSensorMode(static_cast<SensorMode>(value));
            SensorMode mode = this->pHexapod->GetSensorMode();
            setIntegerParam(SmarPod_SensorMode, static_cast<int>(mode));
            spdlog::info("Set sensor mode to {}", static_cast<int>(mode));
        } else if (function == SmarPod_PivotMode) {
            this->pHexapod->SetPivotMode(static_cast<PivotMode>(value));
            PivotMode mode = this->pHexapod->GetPivotMode();
            setIntegerParam(SmarPod_PivotMode, static_cast<int>(mode));
        } else if (function == SmarPod_FindRefMethod) {
            this->pHexapod->SetFindRefMethod(static_cast<FrefMethod>(value));
            FrefMethod method = this->pHexapod->GetFindRefMethod();
            setIntegerParam(SmarPod_FindRefMethod, static_cast<int>(method));
        } else if (function == SmarPod_FindRefDirX) {
            this->pHexapod->SetFindRefDirection(Axis::X, static_cast<FrefDirection>(value));
            FrefDirection dir = this->pHexapod->GetFindRefDirection(Axis::X);
            setIntegerParam(SmarPod_FindRefDirX, static_cast<int>(dir));
            spdlog::info("Set find reference direction for X axis to {}", static_cast<int>(dir));
        } else if (function == SmarPod_FindRefDirY) {
            this->pHexapod->SetFindRefDirection(Axis::Y, static_cast<FrefDirection>(value));
            FrefDirection dir = this->pHexapod->GetFindRefDirection(Axis::Y);
            setIntegerParam(SmarPod_FindRefDirY, static_cast<int>(dir));
            spdlog::info("Set find reference direction for Y axis to {}", static_cast<int>(dir));
        } else if (function == SmarPod_FindRefDirZ) {
            this->pHexapod->SetFindRefDirection(Axis::Z, static_cast<FrefDirection>(value));
            FrefDirection dir = this->pHexapod->GetFindRefDirection(Axis::Z);
            setIntegerParam(SmarPod_FindRefDirZ, static_cast<int>(dir));
            spdlog::info("Set find reference direction for Z axis to {}", static_cast<int>(dir));
        } else if (function == SmarPod_SpeedControl) {
            double speed;
            getDoubleParam(SmarPod_Speed, &speed);
            this->pHexapod->SetSpeed(speed, value != 0);
            auto [newSpeed, newControl] = this->pHexapod->GetSpeed();
            setDoubleParam(SmarPod_Speed, newSpeed);
            setIntegerParam(SmarPod_SpeedControl, newControl);
            spdlog::info("Set speed to {} with control {}", newSpeed,
                         newControl ? "enabled" : "disabled");
        } else if (function == SmarPod_AccelControl) {
            double accel;
            getDoubleParam(SmarPod_Acceleration, &accel);
            this->pHexapod->SetAcceleration(accel, value != 0);
            auto [newAccel, newControl] = this->pHexapod->GetAcceleration();
            setDoubleParam(SmarPod_Acceleration, newAccel);
            setIntegerParam(SmarPod_AccelControl, newControl);
            spdlog::info("Set acceleration to {} with control {}", newAccel,
                         newControl ? "enabled" : "disabled");
        } else if (function == SmarPod_ProtectAllPoses) {
            for (auto& pose : this->storedPoses) {
                pose->modifyProtectionStatus(value != 0);
            }
        } else if (function < SMARPOD_FIRST_PARAM) {
            status = asynPortDriver::writeInt32(pasynUser, value);
        } else {
            setIntegerParam(function, value);
        }
    } catch (const std::exception& e) {
        spdlog::error("Failed to write {} to param {}: {}", value, paramName, e.what());
        setStringParam(SmarPod_StatusMessage, e.what());
        status = asynError;
    }

    callParamCallbacks();
    return status;
}

/**
 * @brief Handles write events to float parameters
 *
 * @param pasynUser Pointer to asynUser for SmarPod instance
 * @param value Value written to the double parameter
 * @return asynSuccess if write was successful, asynError otherwise
 */
asynStatus SmarPod::writeFloat64(asynUser* pasynUser, epicsFloat64 value) {
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;

    const char* paramName;
    getParamName(function, &paramName);

    try {
        if (function == SmarPod_Speed) {
            int control;
            getIntegerParam(SmarPod_SpeedControl, &control);
            this->pHexapod->SetSpeed(value, control != 0);
            auto [speed, speedControl] = this->pHexapod->GetSpeed();
            setDoubleParam(SmarPod_Speed, speed);
            setIntegerParam(SmarPod_SpeedControl, speedControl);
            spdlog::info("Set speed to {} with control {}", speed,
                         speedControl ? "enabled" : "disabled");
        } else if (function == SmarPod_Acceleration) {
            int control;
            getIntegerParam(SmarPod_AccelControl, &control);
            this->pHexapod->SetAcceleration(value, control != 0);
            auto [accel, accelControl] = this->pHexapod->GetAcceleration();
            setDoubleParam(SmarPod_Acceleration, accel);
            setIntegerParam(SmarPod_AccelControl, accelControl);
            spdlog::info("Set acceleration to {} with control {}", accel,
                         accelControl ? "enabled" : "disabled");
        } else if (function == SmarPod_FindRefAndCalibFreq) {
            this->pHexapod->SetFindRefAndCalibFreq(value);
            double freq = this->pHexapod->GetFindRefAndCalibFreq();
            setDoubleParam(SmarPod_FindRefAndCalibFreq, freq);
            spdlog::info("Set find reference and calibration frequency to {}", freq);
        } else if (function == SmarPod_TargetX || function == SmarPod_TargetY ||
                   function == SmarPod_TargetZ || function == SmarPod_TargetRx ||
                   function == SmarPod_TargetRy || function == SmarPod_TargetRz) {
            setDoubleParam(function, value);
            this->checkTargetPose();
        } else if (function < SMARPOD_FIRST_PARAM) {
            status = asynPortDriver::writeFloat64(pasynUser, value);
        } else {
            setDoubleParam(function, value);
        }
        // Pivot/target/coordinate-system/axes setpoints are staged and applied
        // by their corresponding command records.
    } catch (const std::exception& e) {
        spdlog::error("Failed to write {} to param {}: {}", value, paramName, e.what());
        setStringParam(SmarPod_StatusMessage, e.what());
        status = asynError;
    }

    callParamCallbacks();
    return status;
}

/**
 * @brief Handles read events for integer parameters.
 *
 * Refreshes the current pose and move status when the periodic CheckPose poll
 * (or a move status read) is processed.
 *
 * @param pasynUser Pointer to asynUser for SmarPod instance
 * @param value Returned parameter value
 * @return asynSuccess if read was successful, asynError otherwise
 */
asynStatus SmarPod::readInt32(asynUser* pasynUser, epicsInt32* value) {
    int function = pasynUser->reason;

    const char* paramName;
    getParamName(function, &paramName);

    if (function == SmarPod_CheckPose || function == SmarPod_MoveStatus) {
        try {
            this->updatePoseAndStatus();
        } catch (const std::exception& e) {
            spdlog::error("Failed to read param {}: {}", paramName, e.what());
            setStringParam(SmarPod_StatusMessage, e.what());
        }
    }

    return asynPortDriver::readInt32(pasynUser, value);
}

/**
 * @brief Writes a log message into the StatusMessage PV.
 *
 * Safe to call from any thread; takes the asyn port lock before updating the parameter.
 */
void SmarPod::setStatusMessage(const std::string& message) {
    this->lock();
    setStringParam(SmarPod_StatusMessage, message.c_str());
    callParamCallbacks();
    this->unlock();
}

/**
 * @brief Returns the most recently read pose (as published in the Pose PVs).
 */
Pose SmarPod::getCurrentPose() {
    Pose pose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    this->lock();
    getDoubleParam(SmarPod_PoseX, &pose.x);
    getDoubleParam(SmarPod_PoseY, &pose.y);
    getDoubleParam(SmarPod_PoseZ, &pose.z);
    getDoubleParam(SmarPod_PoseRx, &pose.rx);
    getDoubleParam(SmarPod_PoseRy, &pose.ry);
    getDoubleParam(SmarPod_PoseRz, &pose.rz);
    this->unlock();
    return pose;
}

Pose SmarPod::getCurrentTargetPose() {
    Pose target;
    this->lock();
    getDoubleParam(SmarPod_TargetX, &target.x);
    getDoubleParam(SmarPod_TargetY, &target.y);
    getDoubleParam(SmarPod_TargetZ, &target.z);
    getDoubleParam(SmarPod_TargetRx, &target.rx);
    getDoubleParam(SmarPod_TargetRy, &target.ry);
    getDoubleParam(SmarPod_TargetRz, &target.rz);
    this->unlock();
    return target;
}

void SmarPod::setTargetPose(const Pose& target) {
    this->lock();
    setDoubleParam(SmarPod_TargetX, target.x);
    setDoubleParam(SmarPod_TargetY, target.y);
    setDoubleParam(SmarPod_TargetZ, target.z);
    setDoubleParam(SmarPod_TargetRx, target.rx);
    setDoubleParam(SmarPod_TargetRy, target.ry);
    setDoubleParam(SmarPod_TargetRz, target.rz);
    this->unlock();
    this->checkTargetPose();
}

void SmarPod::checkTargetPose() {
    Pose target = getCurrentTargetPose();
    if (!this->pHexapod->IsPoseReachable(target)) {
        spdlog::error("Target pose ({}, {}, {}, {}, {}, {}) is not reachable", target.x, target.y,
                      target.z, target.rx, target.ry, target.rz);
        setIntegerParam(SmarPod_PoseNotReachable, 1);
    } else {
        setIntegerParam(SmarPod_PoseNotReachable, 0);
    }
    callParamCallbacks();
}

void SmarPod::moveToTargetPose() {
    int reachable, holdTime;
    getIntegerParam(SmarPod_PoseNotReachable, &reachable);
    if (reachable) {
        spdlog::error("Cannot move to target pose because it is not reachable!");
        return;
    }
    getIntegerParam(SmarPod_HoldTime, &holdTime);
    Pose target = getCurrentTargetPose();
    this->pHexapod->Move(target, static_cast<unsigned int>(holdTime), 1);
}

/**
 * @brief Reads the current pose and move status from the device and updates the
 * associated parameters.
 */
void SmarPod::updatePoseAndStatus() {
    // Move status is available regardless of reference state.
    MoveStatus moveStatus = this->pHexapod->GetMoveStatus();
    if (moveStatus != MoveStatus::STOPPED) {
        spdlog::debug("Current move status: {}", static_cast<int>(moveStatus));
    }
    setIntegerParam(SmarPod_MoveStatus, static_cast<int>(moveStatus));

    // The pose can only be read once the SmarPod has been referenced.
    if (this->pHexapod->IsReferenced()) {
        spdlog::debug("Reading current pose from SmarPod");
        Pose pose = this->pHexapod->GetPose();
        setDoubleParam(SmarPod_PoseX, pose.x);
        setDoubleParam(SmarPod_PoseY, pose.y);
        setDoubleParam(SmarPod_PoseZ, pose.z);
        setDoubleParam(SmarPod_PoseRx, pose.rx);
        setDoubleParam(SmarPod_PoseRy, pose.ry);
        setDoubleParam(SmarPod_PoseRz, pose.rz);
    } else {
        spdlog::debug("SmarPod is not referenced; skipping pose read");
    }
    callParamCallbacks();
}

/**
 * @brief Function used for reporting SmarPod info to a log.
 *
 * @param fp Open file pointer to log file
 * @param details Level of details to write to the file
 */
void SmarPod::report(FILE* fp, int details) {
    // Low detail: identity
    std::string modelName, modelNumber, sdkVersion, driverVersion, locator;
    getStringParam(SmarPod_ModelName, modelName);
    getStringParam(SmarPod_ModelNumber, modelNumber);
    getStringParam(SmarPod_SdkVersion, sdkVersion);
    getStringParam(SmarPod_DriverVersion, driverVersion);
    getStringParam(SmarPod_Locator, locator);

    fprintf(fp, "SmarPod driver report for port %s\n", this->portName);
    fprintf(fp, "  Model:          %s (model number %s)\n", modelName.c_str(), modelNumber.c_str());
    fprintf(fp, "  Locator:        %s\n", locator.c_str());
    fprintf(fp, "  Driver version: %s\n", driverVersion.c_str());
    fprintf(fp, "  SDK version:    %s\n", sdkVersion.c_str());

    // Medium detail: current pose (only valid once referenced)
    if (details >= 1) {
        if (this->pHexapod->IsReferenced()) {
            Pose pose = this->pHexapod->GetPose();
            fprintf(fp, "  Current pose:   X=%.6f Y=%.6f Z=%.6f m  RX=%.6f RY=%.6f RZ=%.6f deg\n",
                    pose.x, pose.y, pose.z, pose.rx, pose.ry, pose.rz);
        } else {
            fprintf(fp, "  Current pose:   (not referenced)\n");
        }
    }

    // High detail: motion settings and state
    if (details >= 2) {
        auto [speed, speedControl] = this->pHexapod->GetSpeed();
        auto [accel, accelControl] = this->pHexapod->GetAcceleration();
        fprintf(fp, "  Referenced:     %s\n", this->pHexapod->IsReferenced() ? "yes" : "no");
        fprintf(fp, "  Move status:    %d\n", static_cast<int>(this->pHexapod->GetMoveStatus()));
        fprintf(fp, "  Sensor mode:    %d\n", static_cast<int>(this->pHexapod->GetSensorMode()));
        fprintf(fp, "  Speed:          %.6f m/s (control %s)\n", speed,
                speedControl ? "on" : "off");
        fprintf(fp, "  Acceleration:   %.6f m/s^2 (control %s)\n", accel,
                accelControl ? "on" : "off");
        fprintf(fp, "  Max frequency:  %u Hz\n", this->pHexapod->GetMaxFrequency());
    }

    // Very high detail: geometry, referencing config
    if (details >= 3) {
        auto [px, py, pz] = this->pHexapod->GetPivot();
        auto [arx, ary, arz] = this->pHexapod->GetAxesOrientation();
        fprintf(fp, "  Pivot:          X=%.6f Y=%.6f Z=%.6f m (mode %d)\n", px, py, pz,
                static_cast<int>(this->pHexapod->GetPivotMode()));
        fprintf(fp, "  Axes orient.:   RX=%.6f RY=%.6f RZ=%.6f deg\n", arx, ary, arz);
        fprintf(fp, "  Find-ref method: %d, find-ref/calib freq: %.3f Hz\n",
                static_cast<int>(this->pHexapod->GetFindRefMethod()),
                this->pHexapod->GetFindRefAndCalibFreq());
    }
    asynPortDriver::report(fp, details);
}

void SmarPod::getInitialState() {
    // Get initial state of the SmarPod and set parameters accordingly
    auto [speed, speedControl] = this->pHexapod->GetSpeed();
    setDoubleParam(SmarPod_Speed, speed * 1000.0);  // Convert from m/s to mm/s for PV
    setIntegerParam(SmarPod_SpeedControl, static_cast<int>(speedControl));
    auto [accel, accelControl] = this->pHexapod->GetAcceleration();
    setDoubleParam(SmarPod_Acceleration, accel * 1000.0);  // Convert from m/s^2 to mm/s^2 for PV
    setIntegerParam(SmarPod_AccelControl, static_cast<int>(accelControl));
    bool referenced = this->pHexapod->IsReferenced();
    setIntegerParam(SmarPod_IsReferenced, static_cast<int>(referenced));
    setIntegerParam(SmarPod_MaxFrequency, static_cast<int>(this->pHexapod->GetMaxFrequency()));
    setDoubleParam(SmarPod_FindRefAndCalibFreq, this->pHexapod->GetFindRefAndCalibFreq());
    setIntegerParam(SmarPod_MoveStatus, static_cast<int>(this->pHexapod->GetMoveStatus()));
    setIntegerParam(SmarPod_SensorMode, static_cast<int>(this->pHexapod->GetSensorMode()));
    auto [x, y, z] = this->pHexapod->GetPivot();
    setDoubleParam(SmarPod_PivotX, x);
    setDoubleParam(SmarPod_PivotY, y);
    setDoubleParam(SmarPod_PivotZ, z);
    if (referenced) {
        Pose pose = this->pHexapod->GetPose();
        setDoubleParam(SmarPod_PoseX, pose.x);
        setDoubleParam(SmarPod_PoseY, pose.y);
        setDoubleParam(SmarPod_PoseZ, pose.z);
        setDoubleParam(SmarPod_PoseRx, pose.rx);
        setDoubleParam(SmarPod_PoseRy, pose.ry);
        setDoubleParam(SmarPod_PoseRz, pose.rz);

        // If we're referenced, assume we're calibrated as well
        // (the SmarPod API doesn't provide a way to check this)
        setIntegerParam(SmarPod_IsCalibrated, 1);
    }
    auto [rx, ry, rz] = this->pHexapod->GetAxesOrientation();
    setDoubleParam(SmarPod_AxesRx, rx);
    setDoubleParam(SmarPod_AxesRy, ry);
    setDoubleParam(SmarPod_AxesRz, rz);

    SensorMode sensorMode = this->pHexapod->GetSensorMode();
    setIntegerParam(SmarPod_SensorMode, static_cast<int>(sensorMode));
    PivotMode pivotMode = this->pHexapod->GetPivotMode();
    setIntegerParam(SmarPod_PivotMode, static_cast<int>(pivotMode));
    FrefMethod findRefMethod = this->pHexapod->GetFindRefMethod();
    setIntegerParam(SmarPod_FindRefMethod, static_cast<int>(findRefMethod));
    setIntegerParam(SmarPod_FindRefDirX,
                    static_cast<int>(this->pHexapod->GetFindRefDirection(Axis::X)));
    setIntegerParam(SmarPod_FindRefDirY,
                    static_cast<int>(this->pHexapod->GetFindRefDirection(Axis::Y)));
    setIntegerParam(SmarPod_FindRefDirZ,
                    static_cast<int>(this->pHexapod->GetFindRefDirection(Axis::Z)));

    setDoubleParam(SmarPod_TargetX, 0.0);
    setDoubleParam(SmarPod_TargetY, 0.0);
    setDoubleParam(SmarPod_TargetZ, 0.0);
    setDoubleParam(SmarPod_TargetRx, 0.0);
    setDoubleParam(SmarPod_TargetRy, 0.0);
    setDoubleParam(SmarPod_TargetRz, 0.0);

    Pose csys = this->pHexapod->GetCoordinateSystem();
    setDoubleParam(SmarPod_CoordSysX, csys.x);
    setDoubleParam(SmarPod_CoordSysY, csys.y);
    setDoubleParam(SmarPod_CoordSysZ, csys.z);
    setDoubleParam(SmarPod_CoordSysRx, csys.rx);
    setDoubleParam(SmarPod_CoordSysRy, csys.ry);
    setDoubleParam(SmarPod_CoordSysRz, csys.rz);

    setIntegerParam(SmarPod_HoldTime, 0);
    callParamCallbacks();
}

/**
 * @brief Constructor for SmarPod
 *
 * Responsible for initial connection to the device - instance initialized in
 * SmarPodConfig, called at IOC startup.
 *
 * @param portName Asyn port name for the SmarPod object instance.
 * @param locator Locator string for the SmarPod device.
 * @param modelNumber Model number of the SmarPod device.

 */
SmarPod::SmarPod(const char* portName, const char* locator, int modelNumber)

    : asynPortDriver(
          portName, 1, /* maxAddr */
          (int) NUM_SMARPOD_PARAMS,
          asynInt32Mask | asynFloat64Mask | asynFloat64ArrayMask | asynDrvUserMask |
              asynOctetMask, /* Interface mask */
          asynInt32Mask | asynFloat64Mask | asynFloat64ArrayMask |
              asynOctetMask, /* Interrupt mask */
          0, /* asynFlags.  This driver does not block and it is not multi-device, so flag is 0 */
          1, /* Autoconnect */
          0, /* Default priority */
          0) /* Default stack size*/
{
    // Default log level to INFO
    spdlog::set_level(spdlog::level::info);

    this->createAllParams();

    // Mirror all spdlog output into the StatusMessage PV
    this->statusSink = std::make_shared<StatusMessageSink>(this);
    spdlog::default_logger()->sinks().push_back(this->statusSink);

    if (isLocatorSim(locator)) {
        spdlog::info("Using simulated hexapod API");
        this->pApi = std::make_unique<SimHexapodAPI>(locator);
    } else {
#ifdef WITH_SMARPOD
        spdlog::info("Initializing SmarPod API");
        this->pApi = std::make_unique<SmarPodAPI>();
#else
        spdlog::warn(
            "SmarPod API not available. Falling back to simulated hexapod API. Please build with "
            "WITH_SMARPOD defined and the SmarPod SDK installed.");
        this->pApi = std::make_unique<SimHexapodAPI>(locator);
#endif
    }

    // Set the driver version and SDK version parameters
    std::string driverVersion = std::to_string(SMARPOD_VERSION_MAJOR) + "." +
                                std::to_string(SMARPOD_VERSION_MINOR) + "." +
                                std::to_string(SMARPOD_VERSION_PATCH);
    setStringParam(SmarPod_DriverVersion, driverVersion.c_str());
    std::string sdkVersion = this->pApi->GetVersionStr();
    setStringParam(SmarPod_SdkVersion, sdkVersion.c_str());
    spdlog::info("Driver version: {}, SDK version: {}", driverVersion, sdkVersion);

    setStringParam(SmarPod_Locator, locator);

    if (!this->pApi->AreModelAndLocatorValid(modelNumber, locator))
        throw std::runtime_error("Model number " + std::to_string(modelNumber) +
                                 " is not supported, or system locator " + locator +
                                 " could not be found.");

    // TODO: This should probably not be done on every startup, since
    // after this call, a calibration is required.
    // spdlog::info("Configuring controller for model {} at locator {}", modelNumber, locator);
    // try {
    //     this->pApi->ConfigureController(modelNumber, locator);
    //     setIntegerParam(SmarPod_ControllerConfigured, 1);
    // } catch (const std::exception& e) {
    //     spdlog::error("Error configuring controller: {}", e.what());
    //     setIntegerParam(SmarPod_ControllerConfigured, 0);
    //     return;
    // }

    spdlog::info("Opening hexapod for model {} at locator {}", modelNumber, locator);
    this->pHexapod = this->pApi->Open(modelNumber, locator);

    std::string modelName = this->pApi->GetModelName(modelNumber);
    setStringParam(SmarPod_ModelName, modelName.c_str());
    setStringParam(SmarPod_ModelNumber, std::to_string(modelNumber).c_str());

    spdlog::info("Connected to hexapod model {}", modelName);
    this->getInitialState();

    // Create companion drivers managing 10 stored pose slots
    for (int i = 1; i <= 10; i++) {
        std::string spPort = std::string(portName) + "_POSE" + std::to_string(i);
        this->storedPoses.push_back(std::make_unique<SmarPodStoredPose>(spPort.c_str(), this));
    }

    // Set 18.5 kHz for speed-controlled move, and speed to 2 mm/s
    this->pHexapod->SetMaxFrequency(18500);
    this->pHexapod->SetSpeed(0.002, true);

    // Set reference frequency to 8 kHz, which is the default for SmarPod devices
    this->pHexapod->SetFindRefAndCalibFreq(8000);

    callParamCallbacks();

    // When epics is exited, delete the instance of this class
    epicsAtExit(exitCallbackC, (void*) this);
}

/**
 * @brief Destructor for SmarPod
 *
 * Called at IOC exit.
 */
SmarPod::~SmarPod() {
    // Destroy the companion stored-pose drivers before tearing down the parent
    this->storedPoses.clear();
    // Stop routing logs into this (soon-to-be-destroyed) instance's PV first
    if (this->statusSink) {
        auto& sinks = spdlog::default_logger()->sinks();
        sinks.erase(std::remove(sinks.begin(), sinks.end(), this->statusSink), sinks.end());
        this->statusSink.reset();
    }
    spdlog::info("Disconnecting SmarPod...");
    this->pApi->Close(this->pHexapod);
    spdlog::info("Shutdown complete.");
}

//-------------------------------------------------------------
// SmarPod ioc shell registration
//-------------------------------------------------------------

/* SmarPodConfig -> These are the args passed to the constructor in the epics config function */
static const iocshArg SmarPodConfigArg0 = {"portName", iocshArgString};
static const iocshArg SmarPodConfigArg1 = {"locator", iocshArgString};
static const iocshArg SmarPodConfigArg2 = {"modelNumber", iocshArgInt};

/* Array of config args */

static const iocshArg* const SmarPodConfigArgs[] = {&SmarPodConfigArg0, &SmarPodConfigArg1,
                                                    &SmarPodConfigArg2};

/**
 * @brief Call function pointer for IOC shell.
 *
 * @param args Array of IOC shell arguments parsed during IOC startup
 */
static void configSmarPodCallFunc(const iocshArgBuf* args) {
    SmarPodConfig(args[0].sval, args[1].sval, args[2].ival);
}

/* information about the configuration function */
static const iocshFuncDef configSmarPod = {"SmarPodConfig", 3, SmarPodConfigArgs};

static const iocshArg SmarPodSetLogLevelArg0 = {"logLevel", iocshArgInt};
static const iocshArg* const SmarPodSetLogLevelArgs[] = {&SmarPodSetLogLevelArg0};

static void configSmarPodSetLogLevelCallFunc(const iocshArgBuf* args) {
    SmarPodSetLogLevel(args[0].ival);
}

static const iocshFuncDef configSmarPodSetLogLevel = {"SmarPodSetLogLevel", 1,
                                                      SmarPodSetLogLevelArgs};

/* IOC register function */
static void SmarPodRegister(void) {
    iocshRegister(&configSmarPod, configSmarPodCallFunc);
    iocshRegister(&configSmarPodSetLogLevel, configSmarPodSetLogLevelCallFunc);
}

/* external function for IOC register */
extern "C" {
epicsExportRegistrar(SmarPodRegister);
}
