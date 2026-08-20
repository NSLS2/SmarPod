TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#

USR_CPPFLAGS += -std=c++17

ifdef SMARPOD_SDK
USR_CPPFLAGS += -DWITH_SMARPOD
USR_INCLUDES += -I$(SMARPOD_SDK_INCLUDE)
USR_LDFLAGS += -L$(SMARPOD_SDK_LIB)
endif

LIBRARY_IOC += drvSmarPod

INC += drvSmarPod.hpp SmarPodParamDefs.hpp IHexapodAPI.hpp SimHexapodAPI.hpp
ifdef SMARPOD_SDK
INC += SmarPodAPI.hpp
endif

LIB_SRCS += drvSmarPod.cpp SimHexapodAPI.cpp SmarPodParamDefs.cpp
ifdef SMARPOD_SDK
LIB_SRCS += SmarPodAPI.cpp
endif


DBD += drvSmarPodSupport.dbd

LIB_LIBS += spdlog

ifdef SMARPOD_SDK
LIB_SYS_LIBS += smarpod
endif

# #=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
