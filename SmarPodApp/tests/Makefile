TOP = ../..
include $(TOP)/configure/CONFIG

#----------------------------------------

USR_CPPFLAGS += -std=c++17
USR_INCLUDES += -I$(TOP)/SmarPodApp/src

TESTPROD_IOC = TestSmarPod

TestSmarPod_SRCS += TestSmarPodMain.cpp
# TestSmarPod_SRCS += SmarPodTestUtils.cpp
TestSmarPod_SRCS += TestSmarPodAPI.cpp

# Add additional test source files here
# TestSmarPod_SRCS +=

TestSmarPod_LIBS += SmarPod gtest gmock $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES
