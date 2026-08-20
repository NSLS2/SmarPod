#ifndef DRVSMARPOD_H
#define DRVSMARPOD_H
#include <asynPortDriver.h>
#include <epicsExport.h>
#include <iocsh.h>
#include <spdlog/sinks/sink.h>
#include <spdlog/spdlog.h>

#include <memory>
#include <vector>

#include "SimHexapodAPI.hpp"

#ifdef WITH_SMARPOD
#include "SmarPodAPI.hpp"
#endif

#define SMARPOD_VERSION_MAJOR 0
#define SMARPOD_VERSION_MINOR 0
#define SMARPOD_VERSION_PATCH 1

class SmarPodStoredPose;

class SmarPod : public asynPortDriver {
    public:
        SmarPod(const char* portName, const char* locator, int modelNumber);
        ~SmarPod();

        /* These are the methods that we override from asynPortDriver as needed*/
        virtual asynStatus readInt32(asynUser* pasynUser, epicsInt32* value);
        virtual asynStatus writeInt32(asynUser* pasynUser, epicsInt32 value);
        // virtual asynStatus readFloat64(asynUser *pasynUser, epicsFloat64 *value);
        virtual asynStatus writeFloat64(asynUser* pasynUser, epicsFloat64 value);
        // virtual asynStatus readOctet(asynUser *pasynUser, char *value, size_t maxChars, size_t
        // *nActual, int *eomReason); virtual asynStatus writeOctet(asynUser *pasynUser, char
        // *value, size_t maxChars, size_t *nActual, int *eomReason); virtual asynStatus
        // connect(asynUser* pasynUser); virtual asynStatus disconnect(asynUser* pasynUser);
        virtual void report(FILE* fp, int details);

        void createAllParams();
        void getInitialState();

        // Writes a log message into the StatusMessage PV (thread-safe).
        void setStatusMessage(const std::string& message);

        // Accessors used by the SmarPodStoredPose companion driver.
        Pose getCurrentPose();
        void moveToPose(const Pose& pose);

    protected:
#include "SmarPodParamDefs.hpp"

    private:
        std::unique_ptr<IHexapodAPI> pApi;
        std::unique_ptr<Hexapod> pHexapod;
        std::shared_ptr<spdlog::sinks::sink> statusSink;
        std::vector<std::unique_ptr<SmarPodStoredPose>> storedPoses;

        void updatePoseAndStatus();
        epicsThreadId moveThread;
};

#endif
