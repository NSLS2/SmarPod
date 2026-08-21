/**
 * Companion asyn port driver managing a single SmarPod stored pose slot.
 *
 * Copyright (c) : Brookhaven National Laboratory, 2026
 */

#include "SmarPodStoredPose.hpp"

#include <spdlog/spdlog.h>

SmarPodStoredPose::SmarPodStoredPose(const char* portName, SmarPod* parent)
    : asynPortDriver(portName, 1, /* maxAddr */
                     asynInt32Mask | asynFloat64Mask | asynOctetMask | asynDrvUserMask,
                     asynInt32Mask | asynFloat64Mask | asynOctetMask, ASYN_CANBLOCK,
                     1,  /* Autoconnect */
                     0,  /* Default priority */
                     0), /* Default stack size */
      parent(parent) {
    createParam(SmarPodSp_StoredXString, asynParamFloat64, &SmarPodSp_StoredX);
    createParam(SmarPodSp_StoredYString, asynParamFloat64, &SmarPodSp_StoredY);
    createParam(SmarPodSp_StoredZString, asynParamFloat64, &SmarPodSp_StoredZ);
    createParam(SmarPodSp_StoredRxString, asynParamFloat64, &SmarPodSp_StoredRx);
    createParam(SmarPodSp_StoredRyString, asynParamFloat64, &SmarPodSp_StoredRy);
    createParam(SmarPodSp_StoredRzString, asynParamFloat64, &SmarPodSp_StoredRz);
    createParam(SmarPodSp_StoreString, asynParamInt32, &SmarPodSp_Store);
    createParam(SmarPodSp_ClearString, asynParamInt32, &SmarPodSp_Clear);
    createParam(SmarPodSp_MoveString, asynParamInt32, &SmarPodSp_Move);
    createParam(SmarPodSp_ProtectedString, asynParamInt32, &SmarPodSp_Protected);
    createParam(SmarPodPp_PoseName, asynParamOctet, &SmarPodSp_PoseName);

    this->setStoredPose({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    setIntegerParam(SmarPodSp_Protected, 0);
    callParamCallbacks();
}

bool SmarPodStoredPose::isProtected() {
    int prot = 0;
    getIntegerParam(SmarPodSp_Protected, &prot);
    return prot != 0;
}

void SmarPodStoredPose::setStoredPose(const Pose& pose) {
    setDoubleParam(SmarPodSp_StoredX, pose.x);
    setDoubleParam(SmarPodSp_StoredY, pose.y);
    setDoubleParam(SmarPodSp_StoredZ, pose.z);
    setDoubleParam(SmarPodSp_StoredRx, pose.rx);
    setDoubleParam(SmarPodSp_StoredRy, pose.ry);
    setDoubleParam(SmarPodSp_StoredRz, pose.rz);
    callParamCallbacks();
}

void SmarPodStoredPose::storeCurrentPose() {
    this->setStoredPose(this->parent->getCurrentPose());
    spdlog::info("Stored current pose on {}", this->portName);
}

void SmarPodStoredPose::clearStoredPose() {
    this->setStoredPose({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    spdlog::info("Cleared stored pose on {}", this->portName);
}

void SmarPodStoredPose::moveToStoredPose() {
    Pose pose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    getDoubleParam(SmarPodSp_StoredX, &pose.x);
    getDoubleParam(SmarPodSp_StoredY, &pose.y);
    getDoubleParam(SmarPodSp_StoredZ, &pose.z);
    getDoubleParam(SmarPodSp_StoredRx, &pose.rx);
    getDoubleParam(SmarPodSp_StoredRy, &pose.ry);
    getDoubleParam(SmarPodSp_StoredRz, &pose.rz);
    this->parent->setTargetPose(pose);
    spdlog::info("Moving SmarPod to stored pose from {}", this->portName);
    this->parent->spawnMoveThread(moveThread, "MoveToStoredPose");
}

void SmarPodStoredPose::modifyProtectionStatus(bool protect) {
    setIntegerParam(SmarPodSp_Protected, protect ? 1 : 0);
    callParamCallbacks();
}

asynStatus SmarPodStoredPose::writeInt32(asynUser* pasynUser, epicsInt32 value) {
    int function = pasynUser->reason;

    if (function == SmarPodSp_Store) {
        if (value) {
            if (this->isProtected()) {
                spdlog::warn("Stored pose {} is protected; store ignored", this->portName);
                return asynError;
            }
            this->storeCurrentPose();
        }
        return asynSuccess;
    } else if (function == SmarPodSp_Clear) {
        if (value) {
            if (this->isProtected()) {
                spdlog::warn("Stored pose {} is protected; clear ignored", this->portName);
                return asynError;
            }
            this->clearStoredPose();
        }
        return asynSuccess;
    } else if (function == SmarPodSp_Move) {
        if (this->isProtected()) {
            spdlog::warn("Stored pose {} is protected; move ignored", this->portName);
            return asynError;
        }
        if (value) this->moveToStoredPose();
        return asynSuccess;
    }

    // Any other int32 parameters are stored as-is.
    setIntegerParam(function, value);
    callParamCallbacks();
    return asynSuccess;
}

asynStatus SmarPodStoredPose::writeFloat64(asynUser* pasynUser, epicsFloat64 value) {
    int function = pasynUser->reason;

    bool isPoseComponent = (function == SmarPodSp_StoredX || function == SmarPodSp_StoredY ||
                            function == SmarPodSp_StoredZ || function == SmarPodSp_StoredRx ||
                            function == SmarPodSp_StoredRy || function == SmarPodSp_StoredRz);
    if (isPoseComponent && this->isProtected()) {
        spdlog::warn("Stored pose {} is protected; edit ignored", this->portName);
        return asynError;
    }

    setDoubleParam(function, value);
    callParamCallbacks();
    return asynSuccess;
}
