TOP = ../..
include $(TOP)/configure/CONFIG

#----------------------------------------

USR_CPPFLAGS += -std=c++17
USR_INCLUDES += -I$(TOP)/SmarPodApp/src

TESTPROD_IOC = TestSmarPod

TestSmarPod_SRCS += TestSmarPodMain.cpp
# TestSmarPod_SRCS += SmarPodTestUtils.cpp
TestSmarPod_SRCS += TestSimHexapodAPI.cpp
TestSmarPod_SRCS += TestSmarPodMath.cpp

# Add additional test source files here
# TestSmarPod_SRCS +=

TestSmarPod_LIBS += asyn drvSmarPod gtest gmock $(EPICS_BASE_IOC_LIBS)

ifdef SMARPOD_SDK
smarpod_DIR = $(SMARPOD_SDK_LIB)
TestSmarPod_LIBS += smarpod
endif

include $(TOP)/configure/RULES
