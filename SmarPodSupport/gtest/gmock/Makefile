TOP=../../..
include $(TOP)/configure/CONFIG

SRC_DIRS += ../gmock

USR_CPPFLAGS += -std=c++17

LIBRARY_IOC += gmock
INC += gmock/gmock-actions.h
INC += gmock/gmock-cardinalities.h
INC += gmock/gmock-function-mocker.h
INC += gmock/gmock-matchers.h
INC += gmock/gmock.h
INC += gmock/gmock-more-actions.h
INC += gmock/gmock-more-matchers.h
INC += gmock/gmock-nice-strict.h
INC += gmock/gmock-spec-builders.h
INC += gmock/internal/gmock-internal-utils.h
INC += gmock/internal/gmock-pp.h
INC += gmock/internal/gmock-port.h
INC += gmock/internal/custom/gmock-generated-actions.h
INC += gmock/internal/custom/gmock-port.h
INC += gmock/internal/custom/gmock-matchers.h

LIB_SRCS += gmock.cc
LIB_SRCS += gmock-spec-builders.cc
LIB_SRCS += gmock-matchers.cc
LIB_SRCS += gmock-cardinalities.cc
LIB_SRCS += gmock-internal-utils.cc
# LIB_SRCS += gmock-all.cc
# LIB_SRCS += gmock_main.cc

LIB_LIBS += gtest

include $(TOP)/configure/RULES
