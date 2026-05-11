// This file is auto-generated. Do not edit directly.
// Generated from SmarPod.template

#include "drvSmarPod.h"

void SmarPod::createAllParams() {
    createParam(SmarPod_VersionString, asynParamOctet, &SmarPod_Version);
    createParam(SmarPod_SdkVerString, asynParamOctet, &SmarPod_SdkVer);
    createParam(SmarPod_IsReferencedString, asynParamInt32, &SmarPod_IsReferenced);
}
