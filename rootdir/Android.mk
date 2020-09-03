LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE := sprdwl.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_CLASS := ETC
LOCAL_MODULE_PATH := $(TARGET_ROOT_OUT)/lib/modules
LOCAL_SRC_FILES := modules/sprdwl.ko
include $(BUILD_PREBUILT)
