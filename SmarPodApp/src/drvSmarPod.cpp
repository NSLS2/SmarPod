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

#include "drvSmarPod.h"

// Error message formatters
#define ERR(msg) \
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "ERROR | %s::%s: %s\n", driverName, __func__, msg)

#define ERR_ARGS(fmt, ...)                                                                        \
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "ERROR | %s::%s: " fmt "\n", driverName, __func__, \
              __VA_ARGS__);

// Warning message formatters
#define WARN(msg) \
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "WARN | %s::%s: %s\n", driverName, __func__, msg)

#define WARN_ARGS(fmt, ...)                                                                      \
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "WARN | %s::%s: " fmt "\n", driverName, __func__, \
              __VA_ARGS__);

// Log message formatters
#define LOG(msg) \
    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s::%s: %s\n", driverName, __func__, msg)

#define LOG_ARGS(fmt, ...)                                                               \
    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s::%s: " fmt "\n", driverName, __func__, \
              __VA_ARGS__);

using namespace std;

const char* driverName = "SmarPod";

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
extern "C" int SmarPodConfig(const char* portName, const char* ipAddress, int modelNum) {
    new SmarPod(portName, ipAddress, modelNum);

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

// /**
//  * @brief Handles write events to integer parameters
//  *
//  * @param pasynUser Pointer to asynUser for SmarPod instance
//  * @param value Value written to the integer parameter
//  * @return asynSuccess if write was successful, asynError otherwise
//  */
// asynStatus SmarPod::writeInt32(asynUser* pasynUser, epicsInt32 value) {
//     int function = pasynUser->reason;
//     int status = asynSuccess;
//     static const char* functionName = "writeInt32";

//     if (function < FIRST_SMARPOD_PARAM) {
//         status = asynPortDriver::writeInt32(pasynUser, value);
//     }

//     if (status) {
//         ERR_ARGS("ERROR status=%d, function=%d, value=%d", status, function, value);
//         return asynError;
//     } else {  // Don't log period checkStatus PV processing
//         status = setIntegerParam(function, value);
//         LOG_ARGS("function=%d value=%d", function, value);
//     }
//     callParamCallbacks();
//     return asynSuccess;
// }

// /**
//  * @brief Handles write events to float parameters
//  *
//  * @param pasynUser Pointer to asynUser for SmarPod instance
//  * @param value Value written to the double parameter
//  * @return asynSuccess if write was successful, asynError otherwise
//  */
// asynStatus SmarPod::writeFloat64(asynUser* pasynUser, epicsFloat64 value) {
//     int function = pasynUser->reason;
//     asynStatus status = asynSuccess;
//     static const char* functionName = "writeFloat64";
//     if (function < FIRST_SMARPOD_PARAM) {
//         status = asynPortDriver::writeFloat64(pasynUser, value);
//     }

//     if (status) {
//         ERR_ARGS("ERROR status=%d, function=%d, value=%f", status, function, value);
//     } else {
//         status = setDoubleParam(function, value);
//         LOG_ARGS("function=%d value=%f", function, value);
//     }

//     callParamCallbacks();
//     return status;
// }

// /**
//  * @brief Function used for reporting SmarPod info to a log.
//  *
//  * @param fp Open file pointer to log file
//  * @param details Level of details to write to the file
//  */
// void SmarPod::report(FILE* fp, int details) {
//     const char* functionName = "report";
//     LOG("Reporting to external log file");
//     if (details > 0) {
//         fprintf(fp, " Connected Device Information\n");
//         asynPortDriver::report(fp, details);
//     }
// }

// int SmarPod::LogError(Smarpod_Status status)
// {
//     if(status != SMARPOD_OK)
//     {
//         const char *info;
//         if(Smarpod_GetStatusInfo(status,&info))
//             printf("unknown SmarPod status\n");
//         else
//             printf("error: %s\n",info);
//     }
//     return status;
// }

int LogError(Smarpod_Status status) {
    if (status != SMARPOD_OK) {
        const char* info;
        if (Smarpod_GetStatusInfo(status, &info))
            printf("unknown SmarPod status\n");
        else
            printf("error: %s\n", info);
    }
    return status;
}

/**
 * @brief Constructor for SmarPod
 *
 * Responsible for initial connection to the device - instance initialized in
 * SmarPodConfig, called at IOC startup.
 *
 * @param portName Asyn port name for the SmarPod object instance.

 */
SmarPod::SmarPod(const char* portName, const char* ipAddress, int modelNum)

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
    // Create any driver specific parameters
    this->createAllParams();

    unsigned int major, minor, update;
    Smarpod_GetDLLVersion(&major, &minor, &update);
    char drvVer[40], sdkVer[40], locater[40];
    snprintf(drvVer, 40, "%d.%d.%d", DRV_SMARPOD_VERSION_MAJOR, DRV_SMARPOD_VERSION_MINOR,
             DRV_SMARPOD_VERSION_PATCH);
    snprintf(sdkVer, 40, "%u.%u.%u", major, minor, update);
    snprintf(locater, 40, "network:%s", ipAddress);
    printf("using SmarPod library version %s\n", sdkVer);

    setStringParam(SmarPod_Version, drvVer);
    setStringParam(SmarPod_SdkVer, sdkVer);

    printf("Calling Smarpod_Open with args %s, %d\n", locater, modelNum);
    int status = Smarpod_Open(&id, modelNum, locater, "");
    if (status)
        LogError(status);
    else
        printf("Successfully opened SmarPod\n");

    status = Smarpod_SetSensorMode(id, SMARPOD_SENSORS_ENABLED);
    if (status)
        LogError(status);
    else
        printf("Enabled all sensors");

    int referenced;
    Smarpod_IsReferenced(id, &referenced);
    setIntegerParam(SmarPod_IsReferenced, referenced);
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
    printf("Disconnecting SmarPod device...\n");
    int status = Smarpod_Close(id);
    if (status) LogError(status);
    printf("Shutdown complete.\n");
}

//-------------------------------------------------------------
// SmarPod ioc shell registration
//-------------------------------------------------------------

/* SmarPodConfig -> These are the args passed to the constructor in the epics config function */
static const iocshArg SmarPodConfigArg0 = {"portName", iocshArgString};
static const iocshArg SmarPodConfigArg1 = {"ipAddress", iocshArgString};
static const iocshArg SmarPodConfigArg2 = {"model", iocshArgInt};

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

/* IOC register function */
static void SmarPodRegister(void) { iocshRegister(&configSmarPod, configSmarPodCallFunc); }

/* external function for IOC register */
extern "C" {
epicsExportRegistrar(SmarPodRegister);
}
