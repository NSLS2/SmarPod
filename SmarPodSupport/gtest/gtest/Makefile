TOP=../../..

include $(TOP)/configure/CONFIG

SRC_DIRS += ../gtest
USR_CPPFLAGS += -std=c++17

LIBRARY_IOC += gtest

INC += gtest/gtest-assertion-result.h
INC += gtest/gtest-death-test.h
INC += gtest/gtest.h
INC += gtest/gtest-matchers.h
INC += gtest/gtest-message.h
INC += gtest/gtest-param-test.h
INC += gtest/gtest_pred_impl.h
INC += gtest/gtest-printers.h
INC += gtest/gtest_prod.h
INC += gtest/gtest-spi.h
INC += gtest/gtest-test-part.h
INC += gtest/gtest-typed-test.h
INC += gtest/internal/gtest-death-test-internal.h
INC += gtest/internal/gtest-filepath.h
INC += gtest/internal/gtest-internal.h
INC += gtest/internal/gtest-param-util.h
INC += gtest/internal/gtest-port-arch.h
INC += gtest/internal/gtest-port.h
INC += gtest/internal/gtest-string.h
INC += gtest/internal/gtest-type-util.h
INC += gtest/internal/custom/gtest.h
INC += gtest/internal/custom/gtest-port.h
INC += gtest/internal/custom/gtest-printers.h

# LIB_SRCS += gtest-all.cc
LIB_SRCS += gtest-assertion-result.cc
LIB_SRCS += gtest.cc
LIB_SRCS += gtest-death-test.cc
LIB_SRCS += gtest-filepath.cc
# LIB_SRCS += gtest_main.cc
LIB_SRCS += gtest-matchers.cc
LIB_SRCS += gtest-port.cc
LIB_SRCS += gtest-printers.cc
LIB_SRCS += gtest-test-part.cc
LIB_SRCS += gtest-typed-test.cc

include $(TOP)/configure/RULES
