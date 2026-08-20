// This file is auto-generated. Do not edit directly.
// Generated from SmarPod.template

#include "drvSmarPod.hpp"

void SmarPod::createAllParams() {
    createParam(SmarPod_DriverVersionString, asynParamOctet, &SmarPod_DriverVersion);
    createParam(SmarPod_SdkVersionString, asynParamOctet, &SmarPod_SdkVersion);
    createParam(SmarPod_LocatorString, asynParamOctet, &SmarPod_Locator);
    createParam(SmarPod_ModelNumberString, asynParamInt32, &SmarPod_ModelNumber);
    createParam(SmarPod_ModelNameString, asynParamOctet, &SmarPod_ModelName);
    createParam(SmarPod_StatusMessageString, asynParamOctet, &SmarPod_StatusMessage);
    createParam(SmarPod_MaxFrequencyString, asynParamInt32, &SmarPod_MaxFrequency);
    createParam(SmarPod_SpeedString, asynParamFloat64, &SmarPod_Speed);
    createParam(SmarPod_SpeedControlString, asynParamInt32, &SmarPod_SpeedControl);
    createParam(SmarPod_AccelerationString, asynParamFloat64, &SmarPod_Acceleration);
    createParam(SmarPod_AccelControlString, asynParamInt32, &SmarPod_AccelControl);
    createParam(SmarPod_FindReferenceMarksString, asynParamInt32, &SmarPod_FindReferenceMarks);
    createParam(SmarPod_CalibrateString, asynParamInt32, &SmarPod_Calibrate);
    createParam(SmarPod_IsReferencedString, asynParamInt32, &SmarPod_IsReferenced);
    createParam(SmarPod_FindRefAndCalibFreqString, asynParamFloat64, &SmarPod_FindRefAndCalibFreq);
    createParam(SmarPod_FindRefMethodString, asynParamInt32, &SmarPod_FindRefMethod);
    createParam(SmarPod_FindRefDirXString, asynParamInt32, &SmarPod_FindRefDirX);
    createParam(SmarPod_FindRefDirYString, asynParamInt32, &SmarPod_FindRefDirY);
    createParam(SmarPod_FindRefDirZString, asynParamInt32, &SmarPod_FindRefDirZ);
    createParam(SmarPod_SensorModeString, asynParamInt32, &SmarPod_SensorMode);
    createParam(SmarPod_PivotXString, asynParamFloat64, &SmarPod_PivotX);
    createParam(SmarPod_PivotYString, asynParamFloat64, &SmarPod_PivotY);
    createParam(SmarPod_PivotZString, asynParamFloat64, &SmarPod_PivotZ);
    createParam(SmarPod_SetPivotString, asynParamInt32, &SmarPod_SetPivot);
    createParam(SmarPod_PivotModeString, asynParamInt32, &SmarPod_PivotMode);
    createParam(SmarPod_CheckPoseString, asynParamInt32, &SmarPod_CheckPose);
    createParam(SmarPod_PoseXString, asynParamFloat64, &SmarPod_PoseX);
    createParam(SmarPod_PoseYString, asynParamFloat64, &SmarPod_PoseY);
    createParam(SmarPod_PoseZString, asynParamFloat64, &SmarPod_PoseZ);
    createParam(SmarPod_PoseRxString, asynParamFloat64, &SmarPod_PoseRx);
    createParam(SmarPod_PoseRyString, asynParamFloat64, &SmarPod_PoseRy);
    createParam(SmarPod_PoseRzString, asynParamFloat64, &SmarPod_PoseRz);
    createParam(SmarPod_TargetXString, asynParamFloat64, &SmarPod_TargetX);
    createParam(SmarPod_TargetYString, asynParamFloat64, &SmarPod_TargetY);
    createParam(SmarPod_TargetZString, asynParamFloat64, &SmarPod_TargetZ);
    createParam(SmarPod_TargetRxString, asynParamFloat64, &SmarPod_TargetRx);
    createParam(SmarPod_TargetRyString, asynParamFloat64, &SmarPod_TargetRy);
    createParam(SmarPod_TargetRzString, asynParamFloat64, &SmarPod_TargetRz);
    createParam(SmarPod_HoldTimeString, asynParamInt32, &SmarPod_HoldTime);
    createParam(SmarPod_WaitForCompletionString, asynParamInt32, &SmarPod_WaitForCompletion);
    createParam(SmarPod_PoseReachableString, asynParamInt32, &SmarPod_PoseReachable);
    createParam(SmarPod_MoveString, asynParamInt32, &SmarPod_Move);
    createParam(SmarPod_StopString, asynParamInt32, &SmarPod_Stop);
    createParam(SmarPod_StopAndHoldString, asynParamInt32, &SmarPod_StopAndHold);
    createParam(SmarPod_StandbyString, asynParamInt32, &SmarPod_Standby);
    createParam(SmarPod_MoveStatusString, asynParamInt32, &SmarPod_MoveStatus);
    createParam(SmarPod_CoordSysXString, asynParamFloat64, &SmarPod_CoordSysX);
    createParam(SmarPod_CoordSysYString, asynParamFloat64, &SmarPod_CoordSysY);
    createParam(SmarPod_CoordSysZString, asynParamFloat64, &SmarPod_CoordSysZ);
    createParam(SmarPod_CoordSysRxString, asynParamFloat64, &SmarPod_CoordSysRx);
    createParam(SmarPod_CoordSysRyString, asynParamFloat64, &SmarPod_CoordSysRy);
    createParam(SmarPod_CoordSysRzString, asynParamFloat64, &SmarPod_CoordSysRz);
    createParam(SmarPod_SetCoordSysString, asynParamInt32, &SmarPod_SetCoordSys);
    createParam(SmarPod_SetCurrentPoseAsZeroString, asynParamInt32, &SmarPod_SetCurrentPoseAsZero);
    createParam(SmarPod_AxesRxString, asynParamFloat64, &SmarPod_AxesRx);
    createParam(SmarPod_AxesRyString, asynParamFloat64, &SmarPod_AxesRy);
    createParam(SmarPod_AxesRzString, asynParamFloat64, &SmarPod_AxesRz);
    createParam(SmarPod_SetAxesOrientationString, asynParamInt32, &SmarPod_SetAxesOrientation);
    createParam(SmarPod_ProtectAllPosesString, asynParamInt32, &SmarPod_ProtectAllPoses);
    createParam(SmarPod_UnprotectAllPosesString, asynParamInt32, &SmarPod_UnprotectAllPoses);
    createParam(SmarPod_ConfigureSystemString, asynParamInt32, &SmarPod_ConfigureSystem);
}
