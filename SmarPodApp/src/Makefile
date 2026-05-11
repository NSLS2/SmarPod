TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#

USR_CPPFLAGS += -std=c++11
USR_INCLUDES += -I$(SMARPOD_INCLUDE)
USR_LDFLAGS += -L$(SMARPOD_LIB)

LIBRARY_IOC += drvSmarPod

INC += drvSmarPod.hpp

LIB_SRCS += drvSmarPod.cpp

DBD += drvSmarPodSupport.dbd

LIB_SYS_LIBS += smarpod

# #=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE