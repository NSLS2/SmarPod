#!../../bin/linux-x86_64/SmarPodApp

errlogInit(20000)
< envPaths

dbLoadDatabase("$(TOP)/dbd/SmarPodApp.dbd")

SmarPodApp_registerRecordDeviceDriver(pdbbase)

# Define asyn port name
epicsEnvSet("PORT", "DRV1")
epicsEnvSet("PREFIX", "DEV:SP1:")

SmarPodConfig("$(PORT)", "sim:123456", 0)


#asynSetTraceMask("$(PORT)", -1, 0x0)
#asynSetTraceMask("$(PORT)", -1, 0x1)

# Enables both log and error messages
#asynSetTraceMask("$(PORT)", -1, 0x9)
#asynSetTraceMask("$(PORT)", -1, 0xF)
#asynSetTraceMask("$(PORT)", -1, 0x11)
#asynSetTraceMask("$(PORT)", -1, 0xFF)
#asynSetTraceIOMask("$(PORT)", -1, 0x0)
#asynSetTraceIOMask("$(PORT)", -1, 0x2)

dbLoadRecords("$(SMARPOD)/db/SmarPod.template", "P=$(PREFIX), PORT=$(PORT), ADDR=0, TIMEOUT=1")

# Stored poses
dbLoadRecords("$(SMARPOD)/db/SmarPod_StoredPose.template", "P=$(PREFIX), PORT=$(PORT)_POSE1, NUM=1, ADDR=0, TIMEOUT=1")
dbLoadRecords("$(SMARPOD)/db/SmarPod_StoredPose.template", "P=$(PREFIX), PORT=$(PORT)_POSE2, NUM=2, ADDR=0, TIMEOUT=1")
dbLoadRecords("$(SMARPOD)/db/SmarPod_StoredPose.template", "P=$(PREFIX), PORT=$(PORT)_POSE3, NUM=3, ADDR=0, TIMEOUT=1")
dbLoadRecords("$(SMARPOD)/db/SmarPod_StoredPose.template", "P=$(PREFIX), PORT=$(PORT)_POSE4, NUM=4, ADDR=0, TIMEOUT=1")
dbLoadRecords("$(SMARPOD)/db/SmarPod_StoredPose.template", "P=$(PREFIX), PORT=$(PORT)_POSE5, NUM=5, ADDR=0, TIMEOUT=1")
dbLoadRecords("$(SMARPOD)/db/SmarPod_StoredPose.template", "P=$(PREFIX), PORT=$(PORT)_POSE6, NUM=6, ADDR=0, TIMEOUT=1")
dbLoadRecords("$(SMARPOD)/db/SmarPod_StoredPose.template", "P=$(PREFIX), PORT=$(PORT)_POSE7, NUM=7, ADDR=0, TIMEOUT=1")
dbLoadRecords("$(SMARPOD)/db/SmarPod_StoredPose.template", "P=$(PREFIX), PORT=$(PORT)_POSE8, NUM=8, ADDR=0, TIMEOUT=1")
dbLoadRecords("$(SMARPOD)/db/SmarPod_StoredPose.template", "P=$(PREFIX), PORT=$(PORT)_POSE9, NUM=9, ADDR=0, TIMEOUT=1")
dbLoadRecords("$(SMARPOD)/db/SmarPod_StoredPose.template", "P=$(PREFIX), PORT=$(PORT)_POSE10, NUM=10, ADDR=0, TIMEOUT=1")

dbLoadRecords("$(ASYN)/db/asynRecord.db", "P=$(PREFIX), R=AsynIO, PORT=$(PORT), ADDR=0, TIMEOUT=1, OMAX=0, IMAX=0")
dbLoadRecords("$(DEVIOCSTATS)/db/iocAdminSoft.db", "IOC=$(PREFIX)")

iocInit()
