#ifndef DRVSMARPOD_H
#define DRVSMARPOD_H
#include <SmarPod.h>
#include <asynPortDriver.h>
#include <epicsExport.h>
#include <iocsh.h>

#define DRV_SMARPOD_VERSION_MAJOR 0
#define DRV_SMARPOD_VERSION_MINOR 0
#define DRV_SMARPOD_VERSION_PATCH 1

// Defines of strings that map Params to Records
#define SmarPod_VersionString "SMARPOD_VERSION"

class SmarPod : public asynPortDriver {
   public:
    SmarPod(const char* portName, const char* ipAddress, int modelNum);
    ~SmarPod();

    /* These are the methods that we override from asynPortDriver as needed*/
    // virtual asynStatus readInt32(asynUser *pasynUser, epicsInt32 *value);
    //  virtual asynStatus writeInt32(asynUser* pasynUser, epicsInt32 value);
    // virtual asynStatus readFloat64(asynUser *pasynUser, epicsFloat64 *value);
    //  virtual asynStatus writeFloat64(asynUser* pasynUser, epicsFloat64 value);
    // virtual asynStatus readOctet(asynUser *pasynUser, char *value, size_t maxChars, size_t
    // *nActual, int *eomReason); virtual asynStatus writeOctet(asynUser *pasynUser, char *value,
    // size_t maxChars, size_t *nActual, int *eomReason); virtual asynStatus connect(asynUser*
    // pasynUser); virtual asynStatus disconnect(asynUser* pasynUser);
    //  virtual void report(FILE* fp, int details);

   protected:
#include "SmarPodParamDefs.h"

   private:
    unsigned int id;
    void createAllParams();
};

#endif
