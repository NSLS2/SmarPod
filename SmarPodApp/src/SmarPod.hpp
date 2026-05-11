#ifndef SMARPOD_H
#define SMARPOD_H
#include <asynPortDriver.h>
#include <epicsExport.h>
#include <iocsh.h>

#define SMARPOD_VERSION_MAJOR 0
#define SMARPOD_VERSION_MINOR 0
#define SMARPOD_VERSION_PATCH 1



// Defines of strings that map Params to Records
#define SmarPod_VersionString "SMARPOD_VERSION"


class SmarPod : public asynPortDriver {

public:
    SmarPod(const char* portName);
    ~SmarPod();

    /* These are the methods that we override from asynPortDriver as needed*/
    //virtual asynStatus readInt32(asynUser *pasynUser, epicsInt32 *value);
    virtual asynStatus writeInt32(asynUser* pasynUser, epicsInt32 value);
    //virtual asynStatus readFloat64(asynUser *pasynUser, epicsFloat64 *value);
    virtual asynStatus writeFloat64(asynUser* pasynUser, epicsFloat64 value);
    //virtual asynStatus readOctet(asynUser *pasynUser, char *value, size_t maxChars, size_t *nActual, int *eomReason);
    //virtual asynStatus writeOctet(asynUser *pasynUser, char *value, size_t maxChars, size_t *nActual, int *eomReason);
    //virtual asynStatus connect(asynUser* pasynUser);
    //virtual asynStatus disconnect(asynUser* pasynUser);
    virtual void report(FILE* fp, int details);

protected:

#define FIRST_SMARPOD_PARAM SmarPodVersion
    int SmarPod_Version;
#define LAST_SMARPOD_PARAM SmarPodVersion

private:



};

#define NUM_SMARPOD_PARAMS ((int)(&LAST_SMARPOD_PARAM - &FIRST_SMARPOD_PARAM + 1))
#endif