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

#include <cmath>

// EPICS includes
#include <epicsExit.h>
#include <epicsExport.h>
#include <epicsStdio.h>
#include <epicsString.h>
#include <epicsThread.h>
#include <epicsTime.h>
#include <iocsh.h>

#include "drvSmarPod.hpp"



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

extern "C" int SmarPodSetLogLevel(int logLevel) {
    spdlog::set_level(static_cast<spdlog::level::level_enum>(logLevel));
    return (asynSuccess);
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
    setIntegerParam(function, value);

    try {
        if (function == SmarPod_FindReferenceMarks && value) {
            this->pHexapod->FindReferenceMarks();
            this->updatePoseAndStatus();
        } else if (function == SmarPod_Calibrate && value) {
            this->pHexapod->Calibrate();
        } else if (function == SmarPod_Move && value) {
            Pose target;
            getDoubleParam(SmarPod_TargetX, &target.x);
            getDoubleParam(SmarPod_TargetY, &target.y);
            getDoubleParam(SmarPod_TargetZ, &target.z);
            getDoubleParam(SmarPod_TargetRx, &target.rx);
            getDoubleParam(SmarPod_TargetRy, &target.ry);
            getDoubleParam(SmarPod_TargetRz, &target.rz);
            int holdTime, wait;
            getIntegerParam(SmarPod_HoldTime, &holdTime);
            this->pHexapod->Move(target, static_cast<unsigned int>(holdTime), 0);
        } else if (function == SmarPod_Stop && value) {
            this->pHexapod->Stop();
        } else if (function == SmarPod_StopAndHold && value) {
            int holdTime;
            getIntegerParam(SmarPod_HoldTime, &holdTime);
            if (holdTime <= 0){
                this->pHexapod->Stop();
            } else {
                this->pHexapod->StopAndHold(static_cast<unsigned int>(holdTime));
            }
        } else if (function == SmarPod_Standby && value) {
            this->pHexapod->Standby();
        } else if (function == SmarPod_SetPivot && value) {
            double x, y, z;
            getDoubleParam(SmarPod_PivotX, &x);
            getDoubleParam(SmarPod_PivotY, &y);
            getDoubleParam(SmarPod_PivotZ, &z);
            this->pHexapod->SetPivot(x, y, z);
        } else if (function == SmarPod_SetCoordSys && value) {
            Pose csys;
            getDoubleParam(SmarPod_CoordSysX, &csys.x);
            getDoubleParam(SmarPod_CoordSysY, &csys.y);
            getDoubleParam(SmarPod_CoordSysZ, &csys.z);
            getDoubleParam(SmarPod_CoordSysRx, &csys.rx);
            getDoubleParam(SmarPod_CoordSysRy, &csys.ry);
            getDoubleParam(SmarPod_CoordSysRz, &csys.rz);
            this->pHexapod->SetCoordinateSystem(csys);
        } else if (function == SmarPod_SetCurrentPoseAsZero && value) {
            this->pHexapod->SetCurrentPoseAsZero();
        } else if (function == SmarPod_SetAxesOrientation && value) {
            double rx, ry, rz;
            getDoubleParam(SmarPod_AxesRx, &rx);
            getDoubleParam(SmarPod_AxesRy, &ry);
            getDoubleParam(SmarPod_AxesRz, &rz);
            this->pHexapod->SetAxesOrientation(rx, ry, rz);
        } else if (function == SmarPod_MaxFrequency) {
            this->pHexapod->SetMaxFrequency(static_cast<unsigned int>(value));
        } else if (function == SmarPod_SensorMode) {
            this->pHexapod->SetSensorMode(static_cast<SensorMode>(value));
        } else if (function == SmarPod_PivotMode) {
            this->pHexapod->SetPivotMode(static_cast<PivotMode>(value));
        } else if (function == SmarPod_FindRefMethod) {
            this->pHexapod->SetFindRefMethod(static_cast<FrefMethod>(value));
        } else if (function == SmarPod_FindRefDirX) {
            this->pHexapod->SetFindRefDirection(Axis::X, static_cast<FrefDirection>(value));
        } else if (function == SmarPod_FindRefDirY) {
            this->pHexapod->SetFindRefDirection(Axis::Y, static_cast<FrefDirection>(value));
        } else if (function == SmarPod_FindRefDirZ) {
            this->pHexapod->SetFindRefDirection(Axis::Z, static_cast<FrefDirection>(value));
        } else if (function == SmarPod_SpeedControl) {
            double speed;
            getDoubleParam(SmarPod_Speed, &speed);
            this->pHexapod->SetSpeed(speed, value != 0);
        } else if (function == SmarPod_AccelControl) {
            double accel;
            getDoubleParam(SmarPod_Acceleration, &accel);
            this->pHexapod->SetAcceleration(accel, value != 0);
        } else if (function < SMARPOD_FIRST_PARAM) {
            status = asynPortDriver::writeInt32(pasynUser, value);
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
    setDoubleParam(function, value);

    try {
        if (function == SmarPod_Speed) {
            int control;
            getIntegerParam(SmarPod_SpeedControl, &control);
            this->pHexapod->SetSpeed(value, control != 0);
        } else if (function == SmarPod_Acceleration) {
            int control;
            getIntegerParam(SmarPod_AccelControl, &control);
            this->pHexapod->SetAcceleration(value, control != 0);
        } else if (function == SmarPod_FindRefAndCalibFreq) {
            this->pHexapod->SetFindRefAndCalibFreq(value);
        } else if (function < SMARPOD_FIRST_PARAM) {
            status = asynPortDriver::writeFloat64(pasynUser, value);
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
 * @brief Reads the current pose and move status from the device and updates the
 * associated parameters.
 */
void SmarPod::updatePoseAndStatus() {
    // Move status is available regardless of reference state.
    setIntegerParam(SmarPod_MoveStatus, static_cast<int>(this->pHexapod->GetMoveStatus()));

    // The pose can only be read once the SmarPod has been referenced.
    if (this->pHexapod->IsReferenced()) {
        Pose pose = this->pHexapod->GetPose();
        setDoubleParam(SmarPod_PoseX, pose.x);
        setDoubleParam(SmarPod_PoseY, pose.y);
        setDoubleParam(SmarPod_PoseZ, pose.z);
        setDoubleParam(SmarPod_PoseRx, pose.rx);
        setDoubleParam(SmarPod_PoseRy, pose.ry);
        setDoubleParam(SmarPod_PoseRz, pose.rz);
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
    setDoubleParam(SmarPod_Speed, speed);
    setIntegerParam(SmarPod_SpeedControl, static_cast<int>(speedControl));
    auto [accel, accelControl] = this->pHexapod->GetAcceleration();
    setDoubleParam(SmarPod_Acceleration, accel);
    setIntegerParam(SmarPod_AccelControl, static_cast<int>(accelControl));
    setIntegerParam(SmarPod_IsReferenced, static_cast<int>(this->pHexapod->IsReferenced()));
    setIntegerParam(SmarPod_MaxFrequency, static_cast<int>(this->pHexapod->GetMaxFrequency()));
    setDoubleParam(SmarPod_FindRefAndCalibFreq, this->pHexapod->GetFindRefAndCalibFreq());
    setIntegerParam(SmarPod_MoveStatus, static_cast<int>(this->pHexapod->GetMoveStatus()));
    setIntegerParam(SmarPod_SensorMode, static_cast<int>(this->pHexapod->GetSensorMode()));
    auto [x, y, z] = this->pHexapod->GetPivot();
    setDoubleParam(SmarPod_PivotX, x);
    setDoubleParam(SmarPod_PivotY, y);
    setDoubleParam(SmarPod_PivotZ, z);
    if (this->pHexapod->IsReferenced()) {
        Pose pose = this->pHexapod->GetPose();
        setDoubleParam(SmarPod_PoseX, pose.x);
        setDoubleParam(SmarPod_PoseY, pose.y);
        setDoubleParam(SmarPod_PoseZ, pose.z);
        setDoubleParam(SmarPod_PoseRx, pose.rx);
        setDoubleParam(SmarPod_PoseRy, pose.ry);
        setDoubleParam(SmarPod_PoseRz, pose.rz);
    }
    auto [rx, ry, rz] = this->pHexapod->GetAxesOrientation();
    setDoubleParam(SmarPod_AxesRx, rx);
    setDoubleParam(SmarPod_AxesRy, ry);
    setDoubleParam(SmarPod_AxesRz, rz);
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

    if (isLocatorSim(locator)) {
        spdlog::info("Using simulated hexapod API");
        this->pApi = std::make_unique<SimHexapodAPI>(locator);
    } else {
#ifdef WITH_SMARPOD
        spdlog::info("Initializing SmarPod API");
        this->pApi = std::make_unique<SmarPodAPI>();
#else
        spdlog::warning(
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

    spdlog::info("Configuring controller for model {} at locator {}", modelNumber, locator);
    this->pApi->ConfigureController(modelNumber, locator);

    spdlog::info("Opening hexapod for model {} at locator {}", modelNumber, locator);
    this->pHexapod = this->pApi->Open(modelNumber, locator);

    std::string modelName = this->pApi->GetModelName(modelNumber);
    setStringParam(SmarPod_ModelName, modelName.c_str());
    setStringParam(SmarPod_ModelNumber, std::to_string(modelNumber).c_str());

    spdlog::info("Connected to hexapod model {}", modelName);
    this->getInitialState();

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
    const char* functionName = "~SmarPod";
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
