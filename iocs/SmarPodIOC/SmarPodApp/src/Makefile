TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

PROD_NAME = SmarPodApp
PROD_IOC += $(PROD_NAME)

# <name>.dbd will be created from <name>Include.dbd
DBD += $(PROD_NAME).dbd
$(PROD_NAME)_DBD += drvSmarPodSupport.dbd
$(PROD_NAME)_DBD += base.dbd asyn.dbd busySupport.dbd asSupport.dbd devIocStats.dbd calc.dbd calcSupport.dbd

# <name>_registerRecordDeviceDriver.cpp will be created from <name>.dbd
$(PROD_NAME)_SRCS += $(PROD_NAME)_registerRecordDeviceDriver.cpp $(PROD_NAME)Main.cpp

$(PROD_NAME)_LIBS += drvSmarPod asyn busy calc autosave devIocStats $(EPICS_BASE_IOC_LIBS)

ifdef SMARPOD_SDK
smarpod_DIR = $(SMARPOD_SDK_LIB)
$(PROD_NAME)_LIBS += smarpod
endif

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
