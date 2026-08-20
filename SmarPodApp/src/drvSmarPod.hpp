#ifndef DRVSMARPOD_H
#define DRVSMARPOD_H
#include <asynPortDriver.h>
#include <epicsExport.h>
#include <iocsh.h>
#include <spdlog/spdlog.h>

#include <memory>

#include "SimHexapodAPI.hpp"

#ifdef WITH_SMARPOD
#include "SmarPodAPI.hpp"
#endif

#define SMARPOD_VERSION_MAJOR 0
#define SMARPOD_VERSION_MINOR 0
#define SMARPOD_VERSION_PATCH 1

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

    protected:
#include "SmarPodParamDefs.hpp"

    private:
        std::unique_ptr<IHexapodAPI> pApi;
        std::unique_ptr<Hexapod> pHexapod;

        void updatePoseAndStatus();
};

#endif
