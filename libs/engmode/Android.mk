ifneq ($(TARGET_SIMULATOR),true)


LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_PRELINK_MODULE    := false
LOCAL_SHARED_LIBRARIES  := libcutils libsqlite libhardware libhardware_legacy libvbeffect libvbpga libnvexchange libatchannel \
	                       libeng-audio

LOCAL_STATIC_LIBRARIES  :=
LOCAL_LDLIBS        += -Idl
ifeq ($(strip $(BOARD_USE_EMMC)),true)
LOCAL_CFLAGS += -DCONFIG_EMMC
endif

ifeq ($(strip $(TARGET_USERIMAGES_USE_UBIFS)),true)
LOCAL_CFLAGS := -DCONFIG_NAND
endif

ifeq ($(USE_BOOT_AT_DIAG),true)
LOCAL_CFLAGS += -DUSE_BOOT_AT_DIAG
endif
LOCAL_C_INCLUDES    += system/bluetooth/bluedroid/include \
					   hardware/libhardware/include \
					   hardware/libhardware_legacy/include \
                       external/bluetooth/bluedroid/btif/include \
					   system/core/include \
					   external/bluetooth/bluedroid/gki/ulinux \
					   external/bluetooth/bluedroid/stack/include \
					   external/bluetooth/bluedroid/stack/btm \


LOCAL_C_INCLUDES    +=  external/sqlite/dist/
LOCAL_C_INCLUDES    +=  vendor/sprd/open-source/libs/libatchannel/
LOCAL_C_INCLUDES    +=  vendor/sprd/open-source/libs/audio/nv_exchange/
LOCAL_C_INCLUDES    +=  vendor/sprd/open-source/libs/audio/
LOCAL_C_INCLUDES    +=  vendor/sprd/open-source/libs/gps_so/
LOCAL_C_INCLUDES    +=  $(TARGET_OUT_INTERMEDIATES)/KERNEL/source/include/uapi/mtd/
LOCAL_SRC_FILES     := eng_pcclient.c  \
		       eng_diag.c \
		       vlog.c \
		       vdiag.c \
		       eng_productdata.c \
		       adc_calibration.c\
		       crc16.c \
		       eng_attok.c \
		       engopt.c \
		       eng_at.c \
               eng_sqlite.c \
               eng_btwifiaddr.c \
               eng_cmd4linuxhdlr.c \
               eng_testhardware.c \
               power.c \
               backlight.c \
               eng_util.c \
	       eng_autotest.c \
               eng_uevent.c\
               eng_debug.c

ifeq ($(strip $(BOARD_USE_SPRD_4IN1_GPS)),true)
#LOCAL_SRC_FILES     += sprd_gps_eut.c
else
LOCAL_SRC_FILES     += gps_eut.c
endif

ifeq ($(BOARD_HAVE_BLUETOOTH_SPRD), true)
LOCAL_CFLAGS += -DHAS_BLUETOOTH_SPRD
ifneq (,$(filter userdebug eng,$(TARGET_BUILD_VARIANT)))
	LOCAL_CFLAGS += -DCONFIG_BQBTEST
	LOCAL_SHARED_LIBRARIES  += libbt-vendor
	LOCAL_SRC_FILES     += eng_controllerbqbtest.c
endif
endif


ifeq ($(strip $(BOARD_WLAN_DEVICE)),bcmdhd)
LOCAL_CFLAGS += -DENGMODE_EUT_BCM
LOCAL_SRC_FILES     += wifi_eut.c \
                       wifi_eut_pc.c \
				bt_eut.c
ifdef WIFI_DRIVER_FW_PATH_PARAM
LOCAL_CFLAGS += -DWIFI_DRIVER_FW_PATH_PARAM=\"$(WIFI_DRIVER_FW_PATH_PARAM)\"
endif
ifdef WIFI_DRIVER_FW_PATH_MFG
LOCAL_CFLAGS += -DWIFI_DRIVER_FW_PATH_MFG=\"$(WIFI_DRIVER_FW_PATH_MFG)\"
endif
else
LOCAL_CFLAGS += -DENGMODE_EUT_SPRD
LOCAL_SRC_FILES     += wifi_eut_shark.c \
			bt_eut_shark.c
ifdef WIFI_DRIVER_MODULE_PATH
LOCAL_CFLAGS        += -DWIFI_DRIVER_MODULE_PATH=\"$(WIFI_DRIVER_MODULE_PATH)\"
endif
ifdef WIFI_DRIVER_MODULE_NAME
LOCAL_CFLAGS        += -DWIFI_DRIVER_MODULE_NAME=\"$(WIFI_DRIVER_MODULE_NAME)\"
endif
endif

LOCAL_MODULE := engpc
LOCAL_MODULE_TAGS := optional
include $(BUILD_EXECUTABLE)
include $(call all-makefiles-under,$(LOCAL_PATH))
endif
