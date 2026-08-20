#ifndef SMARPOD_STORED_POSE_H
#define SMARPOD_STORED_POSE_H

#include <asynPortDriver.h>

#include "drvSmarPod.hpp"

// Parameter drvInfo strings (shared with SmarPod_StoredPose.template)
#define SmarPodSp_StoredXString "SMARPOD_STORED_POSE_X"
#define SmarPodSp_StoredYString "SMARPOD_STORED_POSE_Y"
#define SmarPodSp_StoredZString "SMARPOD_STORED_POSE_Z"
#define SmarPodSp_StoredRxString "SMARPOD_STORED_POSE_RX"
#define SmarPodSp_StoredRyString "SMARPOD_STORED_POSE_RY"
#define SmarPodSp_StoredRzString "SMARPOD_STORED_POSE_RZ"
#define SmarPodSp_StoreString "SMARPOD_STORE_POSE"
#define SmarPodSp_ClearString "SMARPOD_CLEAR_POSE"
#define SmarPodSp_MoveString "SMARPOD_MOVE_POSE"
#define SmarPodSp_ProtectString "SMARPOD_PROTECT_POSE"
#define SmarPodSp_ProtectedString "SMARPOD_POSE_PROTECTED"
#define SmarPodPp_PoseName "SMARPOD_POSE_NAME"

/**
 * @brief Companion asyn port driver that manages a single stored pose slot.
 *
 * Each instance owns one pose (six components), backed by a parent SmarPod
 * driver. Store captures the parent's current pose, Clear zeroes it, Move
 * commands the parent to the stored pose, and Protect locks the slot against
 * store/clear/direct edits.
 */
class SmarPodStoredPose : public asynPortDriver {
    public:
        SmarPodStoredPose(const char* portName, SmarPod* parent);

        virtual asynStatus writeInt32(asynUser* pasynUser, epicsInt32 value);
        virtual asynStatus writeFloat64(asynUser* pasynUser, epicsFloat64 value);

    protected:
        int SmarPodSp_StoredX;
        int SmarPodSp_StoredY;
        int SmarPodSp_StoredZ;
        int SmarPodSp_StoredRx;
        int SmarPodSp_StoredRy;
        int SmarPodSp_StoredRz;
        int SmarPodSp_Store;
        int SmarPodSp_Clear;
        int SmarPodSp_Move;
        int SmarPodSp_Protect;
        int SmarPodSp_Protected;
        int SmarPodSp_PoseName;

    private:
        SmarPod* parent;

        bool isProtected();
        void setStoredPose(const Pose& pose);
        void storeCurrentPose();
        void clearStoredPose();
        void moveToStoredPose();
};

#endif
