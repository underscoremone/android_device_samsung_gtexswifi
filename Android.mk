LOCAL_PATH := $(call my-dir)

ifeq ($(TARGET_DEVICE),gtexswifi)
include $(call all-subdir-makefiles,$(LOCAL_PATH))
endif
