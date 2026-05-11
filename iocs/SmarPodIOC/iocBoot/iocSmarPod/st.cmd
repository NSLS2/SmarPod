#!../../bin/linux-x86_64/SmarPodApp

errlogInit(20000)
< envPaths


dbLoadDatabase("$(TOP)/dbd/SmarPodApp.dbd")

SmarPodApp_registerRecordDeviceDriver(pdbbase)

# Define asyn port name
epicsEnvSet("PORT", "SP1")
epicsEnvSet("PREFIX", "DEV:SP1:")

SmarPodConfig("$(PORT)", "10.69.58.99", 10001)


#asynSetTraceMask("$(PORT)", -1, 0x0)
#asynSetTraceMask("$(PORT)", -1, 0x1)

# Enables both log and error messages
asynSetTraceMask("$(PORT)", -1, 0x9)
#asynSetTraceMask("$(PORT)", -1, 0xF)
#asynSetTraceMask("$(PORT)", -1, 0x11)
#asynSetTraceMask("$(PORT)", -1, 0xFF)
#asynSetTraceIOMask("$(PORT)", -1, 0x0)
#asynSetTraceIOMask("$(PORT)", -1, 0x2)

dbLoadRecords("$(SMARPOD)/db/SmarPod.template", "PREFIX=$(PREFIX), PORT=$(PORT), ADDR=0, TIMEOUT=1")
iocInit()

#dbLoadRecords("$(ASYN)/db/asynRecord.template", "P=$(PREFIX), R=AsynIO, ADDR=0, TIMEOUT=1, OMAX=0, IMAX=0")
#dbLoadRecords("$(DEVIOCSTATS)/db/iocAdminSoft.db", "IOC=$(PREFIX)")
