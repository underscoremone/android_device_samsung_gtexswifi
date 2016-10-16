#leon liu added
#Android makefile for iw3.8
LOCAL_PATH := $(call my-dir)

#source files
IWNPI_OBJS	  += iwnpi.c util.c cmd.c
#cflags
IWNPI_CFLAGS ?= -O2 -g
IWNPI_CFLAGS += -Wall -Wundef -Wstrict-prototypes -Wno-trigraphs -fno-strict-aliasing -fno-common -Werror-implicit-function-declaration -DCONFIG_LIBNL20
#include dirs
INCLUDES += external/libnl-headers

#Build iwnpi tool
include $(CLEAR_VARS)
LOCAL_MODULE := iwnpi
LOCAL_MODULE_TAGS := debug 
LOCAL_CFLAGS = $(IWNPI_CFLAGS)
LOCAL_SRC_FILES = $(IWNPI_OBJS)
LOCAL_C_INCLUDES = $(INCLUDES)
LOCAL_STATIC_LIBRARIES += libnl_2
include $(BUILD_EXECUTABLE)


#Build libiwnpi
CAL_PATH:= $(call my-dir)
include $(CLEAR_VARS)

LOCAL_PRELINK_MODULE := false

LIBIWNPI_OBJS	  += iwnpi_cmd_executer.c util.c cmd.c
#cflags
LIBIWNPI_CFLAGS ?= -O2 -g
LIBIWNPI_CFLAGS += -Wall -Wundef -Wstrict-prototypes -Wno-trigraphs -fno-strict-aliasing -fno-common -Werror-implicit-function-declaration -DCONFIG_LIBNL20
#include dirs
INCLUDES += external/libnl-headers

LOCAL_MODULE := libiwnpi
LOCAL_MODULE_TAGS := debug
LOCAL_CFLAGS = $(LIBIWNPI_CFLAGS)
LOCAL_SRC_FILES = $(LIBIWNPI_OBJS)
LOCAL_C_INCLUDES = $(INCLUDES)
LOCAL_STATIC_LIBRARIES += libnl_2
include $(BUILD_SHARED_LIBRARY)

