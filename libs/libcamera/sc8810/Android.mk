LOCAL_PATH:= $(call my-dir)
include $(CLEAR_VARS)

ifeq ($(strip $(TARGET_BOARD_PLATFORM)),sc8810)

# When zero we link against libqcamera; when 1, we dlopen libqcamera.
DLOPEN_LIBQCAMERA:= 1

LOCAL_C_INCLUDES := \
	$(LOCAL_PATH) \
        $(LOCAL_PATH)/sc8810 \
	$(LOCAL_PATH)/vsp/sc8810/inc	\
	$(LOCAL_PATH)/vsp/sc8810/src \
	$(LOCAL_PATH)/jpeg_fw_8810/inc \
	$(LOCAL_PATH)/jpeg_fw_8810/src \
	external/skia/include/images \
	external/skia/include/core\
        external/jhead \
        external/sqlite/dist \
    system/media/camera/include \
	$(TOP)/hardware/sprd/kernel_headers/sc8810 \
	$(TOP)/hardware/sprd/gralloc/sc8810 \
	$(TOP)/hardware/sprd/mali/sc8810/src/ump/include \

LOCAL_SRC_FILES:= \
	sc8810/SprdOEMCamera.cpp \
        sc8810/SprdCameraHardwareInterface.cpp \
	vsp/sc8810/src/vsp_drv_sc8810.c \
	jpeg_fw_8810/src/jpegcodec_bufmgr.c \
	jpeg_fw_8810/src/jpegcodec_global.c \
	jpeg_fw_8810/src/jpegcodec_table.c \
	jpeg_fw_8810/src/jpegenc_bitstream.c \
	jpeg_fw_8810/src/jpegenc_frame.c \
	jpeg_fw_8810/src/jpegenc_header.c \
	jpeg_fw_8810/src/jpegenc_init.c \
	jpeg_fw_8810/src/jpegenc_interface.c \
	jpeg_fw_8810/src/jpegenc_malloc.c \
	jpeg_fw_8810/src/jpegenc_api.c \
        jpeg_fw_8810/src/jpegdec_bitstream.c \
	jpeg_fw_8810/src/jpegdec_frame.c \
	jpeg_fw_8810/src/jpegdec_init.c \
	jpeg_fw_8810/src/jpegdec_interface.c \
	jpeg_fw_8810/src/jpegdec_malloc.c \
	jpeg_fw_8810/src/jpegdec_dequant.c	\
	jpeg_fw_8810/src/jpegdec_out.c \
	jpeg_fw_8810/src/jpegdec_parse.c \
	jpeg_fw_8810/src/jpegdec_pvld.c \
	jpeg_fw_8810/src/jpegdec_vld.c \
	jpeg_fw_8810/src/jpegdec_api.c \
	jpeg_fw_8810/src/exif_writer.c \
	jpeg_fw_8810/src/jpeg_stream.c


endif

ifeq ($(strip $(TARGET_BOARD_PLATFORM)),sc8825)
LOCAL_C_INCLUDES := \
	$(LOCAL_PATH)/ispvideo	\
	$(LOCAL_PATH)/vsp/sc8825/inc	\
	$(LOCAL_PATH)/vsp/sc8825/src \
	$(LOCAL_PATH)/jpeg_fw_8825/inc \
	$(LOCAL_PATH)/jpeg_fw_8825/src \
	$(LOCAL_PATH)/sc8825/inc \
	$(LOCAL_PATH)/sc8825/isp/inc \
	external/skia/include/images \
	external/skia/include/core\
        external/jhead \
        external/sqlite/dist \
	$(TARGET_OUT_INTERMEDIATES)/KERNEL/source/include/video \
	$(TOP)/device/sprd/common/libs/gralloc \
	$(TOP)/device/sprd/common/libs/mali/src/ump/include

LOCAL_SRC_FILES:= \
	sc8825/src/SprdOEMCamera.c \
        sc8825/src/SprdCameraHardwareInterface.cpp \
	sc8825/src/cmr_oem.c \
	sc8825/src/cmr_set.c \
	sc8825/src/cmr_mem.c \
	sc8825/src/cmr_msg.c \
	sc8825/src/cmr_cvt.c \
	sc8825/src/cmr_v4l2.c \
	sc8825/src/jpeg_codec.c \
	sc8825/src/dc_cfg.c \
	sc8825/src/dc_product_cfg.c \
	sc8825/src/sensor_cfg.c \
	sc8825/src/sensor_drv_u.c \
	sc8825/src/cmr_arith.c \
	sensor/sensor_ov5640_raw.c  \
	sensor/sensor_ov5640.c  \
	sensor/sensor_ov2640.c  \
	sensor/sensor_ov2655.c  \
	sensor/sensor_ov7675.c  \
	sensor/sensor_gc0309.c  \
	sensor/sensor_s5k5ccgx.c \
	sensor/sensor_s5k5ccgx_mipi.c \
	sensor/sensor_ov5640_mipi.c  \
	sensor/sensor_ov5640_mipi_raw.c \
	sensor/sensor_hi351_mipi.c \
	vsp/sc8825/src/vsp_drv_sc8825.c \
	jpeg_fw_8825/src/jpegcodec_bufmgr.c \
	jpeg_fw_8825/src/jpegcodec_global.c \
	jpeg_fw_8825/src/jpegcodec_table.c \
	jpeg_fw_8825/src/jpegenc_bitstream.c \
	jpeg_fw_8825/src/jpegenc_frame.c \
	jpeg_fw_8825/src/jpegenc_header.c \
	jpeg_fw_8825/src/jpegenc_init.c \
	jpeg_fw_8825/src/jpegenc_interface.c \
	jpeg_fw_8825/src/jpegenc_malloc.c \
	jpeg_fw_8825/src/jpegenc_api.c \
        jpeg_fw_8825/src/jpegdec_bitstream.c \
	jpeg_fw_8825/src/jpegdec_frame.c \
	jpeg_fw_8825/src/jpegdec_init.c \
	jpeg_fw_8825/src/jpegdec_interface.c \
	jpeg_fw_8825/src/jpegdec_malloc.c \
	jpeg_fw_8825/src/jpegdec_dequant.c	\
	jpeg_fw_8825/src/jpegdec_out.c \
	jpeg_fw_8825/src/jpegdec_parse.c \
	jpeg_fw_8825/src/jpegdec_pvld.c \
	jpeg_fw_8825/src/jpegdec_vld.c \
	jpeg_fw_8825/src/jpegdec_api.c  \
	jpeg_fw_8825/src/exif_writer.c  \
	jpeg_fw_8825/src/jpeg_stream.c \
	ispvideo/isp_video.c \
	sc8825/src/isp_param_tune_com.c \
	sc8825/src/isp_param_tune_v0000.c
endif

ifeq ($(strip $(TARGET_BOARD_PLATFORM)),sc8830)
LOCAL_C_INCLUDES := \
	$(LOCAL_PATH)/ispvideo	\
	$(LOCAL_PATH)/vsp/sc8825/inc	\
	$(LOCAL_PATH)/vsp/sc8825/src \
	$(LOCAL_PATH)/jpeg_fw_8825/inc \
	$(LOCAL_PATH)/jpeg_fw_8825/src \
	$(LOCAL_PATH)/sc8825/inc \
	$(LOCAL_PATH)/sc8825/isp/inc \
	external/skia/include/images \
	external/skia/include/core\
        external/jhead \
        external/sqlite/dist \
	$(TARGET_OUT_INTERMEDIATES)/KERNEL/source/include/video \
	$(TOP)/device/sprd/common/libs/gralloc \
	$(TOP)/device/sprd/common/libs/mali/src/ump/include

LOCAL_SRC_FILES:= \
	sc8825/src/SprdOEMCamera.c \
        sc8825/src/SprdCameraHardwareInterface.cpp \
	sc8825/src/cmr_oem.c \
	sc8825/src/cmr_set.c \
	sc8825/src/cmr_mem.c \
	sc8825/src/cmr_msg.c \
	sc8825/src/cmr_cvt.c \
	sc8825/src/cmr_v4l2.c \
	sc8825/src/jpeg_codec.c \
	sc8825/src/dc_cfg.c \
	sc8825/src/dc_product_cfg.c \
	sc8825/src/sensor_cfg.c \
	sc8825/src/sensor_drv_u.c \
	sc8825/src/cmr_arith.c \
	sensor/sensor_ov5640_raw.c  \
	sensor/sensor_ov5640.c  \
	sensor/sensor_ov2640.c  \
	sensor/sensor_ov2655.c  \
	sensor/sensor_ov7675.c  \
	sensor/sensor_gc0309.c  \
	sensor/sensor_s5k5ccgx.c \
	sensor/sensor_s5k5ccgx_mipi.c \
	sensor/sensor_ov5640_mipi.c  \
	sensor/sensor_ov5640_mipi_raw.c \
	sensor/sensor_hi351_mipi.c \
	vsp/sc8825/src/vsp_drv_sc8825.c \
	jpeg_fw_8825/src/jpegcodec_bufmgr.c \
	jpeg_fw_8825/src/jpegcodec_global.c \
	jpeg_fw_8825/src/jpegcodec_table.c \
	jpeg_fw_8825/src/jpegenc_bitstream.c \
	jpeg_fw_8825/src/jpegenc_frame.c \
	jpeg_fw_8825/src/jpegenc_header.c \
	jpeg_fw_8825/src/jpegenc_init.c \
	jpeg_fw_8825/src/jpegenc_interface.c \
	jpeg_fw_8825/src/jpegenc_malloc.c \
	jpeg_fw_8825/src/jpegenc_api.c \
        jpeg_fw_8825/src/jpegdec_bitstream.c \
	jpeg_fw_8825/src/jpegdec_frame.c \
	jpeg_fw_8825/src/jpegdec_init.c \
	jpeg_fw_8825/src/jpegdec_interface.c \
	jpeg_fw_8825/src/jpegdec_malloc.c \
	jpeg_fw_8825/src/jpegdec_dequant.c	\
	jpeg_fw_8825/src/jpegdec_out.c \
	jpeg_fw_8825/src/jpegdec_parse.c \
	jpeg_fw_8825/src/jpegdec_pvld.c \
	jpeg_fw_8825/src/jpegdec_vld.c \
	jpeg_fw_8825/src/jpegdec_api.c  \
	jpeg_fw_8825/src/exif_writer.c  \
	jpeg_fw_8825/src/jpeg_stream.c \
	ispvideo/isp_video.c \
	sc8825/src/isp_param_tune_com.c \
	sc8825/src/isp_param_tune_v0000.c

endif

LOCAL_MODULE_PATH := $(TARGET_OUT_SHARED_LIBRARIES)/hw
LOCAL_CFLAGS := -fno-strict-aliasing -D_VSP_ -DJPEG_ENC -D_VSP_LINUX_ -DCHIP_ENDIAN_LITTLE -DCONFIG_CAMERA_2M

ifeq ($(strip $(TARGET_BOARD_PLATFORM)),sc8810)
LOCAL_CFLAGS += -DCONFIG_CAMERA_5M -w
endif

ifeq ($(strip $(CAMERA_SUPPORT_SIZE)),5M)
LOCAL_CFLAGS += -DCONFIG_CAMERA_SUPPORT_5M
endif

ifeq ($(strip $(CAMERA_SUPPORT_SIZE)),3M)
LOCAL_CFLAGS += -DCONFIG_CAMERA_SUPPORT_3M
endif

ifeq ($(strip $(CAMERA_SUPPORT_SIZE)),2M)
LOCAL_CFLAGS += -DCONFIG_CAMERA_SUPPORT_2M
endif

ifeq ($(strip $(CAMERA_SUPPORT_SIZE)),50W)
LOCAL_CFLAGS += -DCONFIG_CAMERA_SUPPORT_50W
endif

ifeq ($(strip $(CAMERA_SUPPORT_SIZE)),130W)
LOCAL_CFLAGS += -DCONFIG_CAMERA_SUPPORT_130W
endif

ifeq ($(strip $(TARGET_BOARD_NO_FRONT_SENSOR)),true)
LOCAL_CFLAGS += -DCONFIG_DCAM_SENSOR_NO_FRONT_SUPPORT
endif

ifeq ($(strip $(TARGET_BOARD_Z788)),true)
LOCAL_CFLAGS += -DCONFIG_CAMERA_788
endif

ifneq ($(strip $(TARGET_BOARD_PLATFORM)),sc8810)
LOCAL_CFLAGS += -DCONFIG_CAMERA_ISP
endif

ifeq ($(strip $(TARGET_BOARD_FRONT_CAMERA_ROTATION)),true)
LOCAL_CFLAGS += -DCONFIG_FRONT_CAMERA_ROTATION
endif

ifeq ($(strip $(TARGET_BOARD_BACK_CAMERA_ROTATION)),true)
LOCAL_CFLAGS += -DCONFIG_BACK_CAMERA_ROTATION
endif

ifeq ($(strip $(CAMERA_DISP_ION)),true)
LOCAL_CFLAGS += -DUSE_ION_MEM
endif

ifeq ($(strip $(TARGET_BOARD_FRONT_CAMERA_SUPPORT)),false)
LOCAL_CFLAGS += -DCONFIG_FRONT_CAMERA_NONE
endif

ifeq ($(strip $(TARGET_BOARD_CAMERA_ROTATION_CAPTURE)),true)
LOCAL_CFLAGS += -DCONFIG_CAMERA_ROTATION_CAPTURE
endif
        
LOCAL_MODULE := camera.$(TARGET_BOARD_PLATFORM)
LOCAL_MODULE_TAGS := optional

ifeq ($(strip $(TARGET_BOARD_PLATFORM)),sc8825)
LOCAL_SHARED_LIBRARIES := libexif libutils libbinder libcamera_client libskia libcutils libsqlite libhardware libisp libmorpho_facesolid libmorpho_easy_hdr
endif

ifeq ($(strip $(TARGET_BOARD_PLATFORM)),sc8810)
LOCAL_SHARED_LIBRARIES := libexif libutils libbinder libmemoryheapion_sprd libcamera_client libskia libcutils libsqlite libhardware
endif

ifeq ($(strip $(TARGET_BOARD_PLATFORM)),sc8830)
LOCAL_SHARED_LIBRARIES := libexif libutils libbinder libcamera_client libskia libcutils libsqlite libhardware libisp libmorpho_facesolid libmorpho_easy_hdr
endif

include $(BUILD_SHARED_LIBRARY)

ifeq ($(strip $(TARGET_BOARD_PLATFORM)),sc8825)

include $(CLEAR_VARS)
LOCAL_PREBUILT_LIBS := sc8825/isp/libisp.so
LOCAL_MODULE_TAGS := optional
include $(BUILD_MULTI_PREBUILT)

include $(CLEAR_VARS)
LOCAL_PREBUILT_LIBS := arithmetic/sc8825/libmorpho_facesolid.so
LOCAL_MODULE_TAGS := optional
include $(BUILD_MULTI_PREBUILT)

include $(CLEAR_VARS)
LOCAL_PREBUILT_LIBS := arithmetic/sc8825/libmorpho_easy_hdr.so
LOCAL_MODULE_TAGS := optional
include $(BUILD_MULTI_PREBUILT)

endif


ifeq ($(strip $(TARGET_BOARD_PLATFORM)),sc8830)

include $(CLEAR_VARS)
LOCAL_PREBUILT_LIBS := sc8825/isp/libisp.so
LOCAL_MODULE_TAGS := optional
include $(BUILD_MULTI_PREBUILT)

include $(CLEAR_VARS)
LOCAL_PREBUILT_LIBS := arithmetic/sc8825/libmorpho_facesolid.so
LOCAL_MODULE_TAGS := optional
include $(BUILD_MULTI_PREBUILT)

include $(CLEAR_VARS)
LOCAL_PREBUILT_LIBS := arithmetic/sc8825/libmorpho_easy_hdr.so
LOCAL_MODULE_TAGS := optional
include $(BUILD_MULTI_PREBUILT)

endif
