#
# Copyright (C) 2008 The Android Open Source Project
#
# Copyright (C) 2016 The CyanogenMod Project
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

LOCAL_PATH := $(call my-dir)

ifeq ($(USE_SPRD_HWCOMPOSER),true)

include $(CLEAR_VARS)

LOCAL_MODULE := hwcomposer.$(TARGET_BOARD_PLATFORM)

LOCAL_MODULE_RELATIVE_PATH := hw

LOCAL_SHARED_LIBRARIES := \
	libion_sprd \
	liblog \
	libEGL \
	libutils \
	libcutils \
	libGLESv1_CM \
	libGLESv2 \
	libhardware \
	libui \
	libsync \
	libmemoryheapion_sprd \

LOCAL_SRC_FILES := \
	SprdHWComposer.cpp \
	SprdPrimaryDisplayDevice/SprdFrameBufferHAL.cpp \
	AndroidFence.cpp \
	SprdDisplayPlane.cpp \
	SprdPrimaryDisplayDevice/SprdPrimaryDisplayDevice.cpp \
	SprdPrimaryDisplayDevice/SprdVsyncEvent.cpp \
	SprdPrimaryDisplayDevice/SprdHWLayerList.cpp \
	SprdHWLayer.cpp \
	SprdPrimaryDisplayDevice/SprdOverlayPlane.cpp \
	SprdPrimaryDisplayDevice/SprdPrimaryPlane.cpp \
	SprdVirtualDisplayDevice/SprdVirtualDisplayDevice.cpp \
	SprdVirtualDisplayDevice/SprdVDLayerList.cpp \
	SprdExternalDisplayDevice/SprdExternalDisplayDevice.cpp \
	SprdUtil.cpp \
	dump.cpp \
	../HWCUtils/Utils.cpp \

LOCAL_C_INCLUDES := \
	$(LOCAL_PATH)/../../gralloc/$(TARGET_BOARD_PLATFORM) \
	$(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr/include/video/ \
	$(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr/include/

LOCAL_ADDITIONAL_DEPENDENCIES += \
	$(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr

LOCAL_CFLAGS += \
	-DLOG_TAG=\"SPRDHWComposer\" \
	-D_USE_SPRD_HWCOMPOSER \
	-DGL_GLEXT_PROTOTYPES \
	-DEGL_EGLEXT_PROTOTYPES \

#DEVICE_OVERLAYPLANE_BORROW_PRIMARYPLANE_BUFFER can make SprdPrimaryPlane
#share the plane buffer to SprdOverlayPlane,
#save 3 screen size YUV420 buffer memory.
#DEVICE_PRIMARYPLANE_USE_RGB565 make the SprdPrimaryPlane use
#RGB565 format buffer to display, also can save 4 screen size
#buffer memory, but it will reduce the image quality.

ifneq (,$(filter sc8830 scx15,$(TARGET_BOARD_PLATFORM)))
DEVICE_WITH_GSP := true
DEVICE_OVERLAYPLANE_BORROW_PRIMARYPLANE_BUFFER := true
endif

ifeq ($(TARGET_BOARD_PLATFORM),sc8830)
DEVICE_DIRECT_DISPLAY_SINGLE_OSD_LAYER := true
DEVICE_USE_FB_HW_VSYNC := true
endif

ifeq ($(TARGET_BOARD_PLATFORM),scx15)
#DEVICE_PRIMARYPLANE_USE_RGB565 := true
#DEVICE_DYNAMIC_RELEASE_PLANEBUFFER := true
endif

ifeq ($(DEVICE_USE_FB_HW_VSYNC),true)
LOCAL_CFLAGS += -DUSE_FB_HW_VSYNC
endif

ifeq ($(DEVICE_WITH_GSP),true)

LOCAL_C_INCLUDES += $(LOCAL_PATH)/../libcamera/sc8830/inc

#LOCAL_CFLAGS += -DVIDEO_LAYER_USE_RGB
# PROCESS_VIDEO_USE_GSP : protecting sc8830 code
LOCAL_CFLAGS += -DPROCESS_VIDEO_USE_GSP
LOCAL_CFLAGS += -DGSP_OUTPUT_USE_YUV420
LOCAL_CFLAGS += -DGSP_SCALING_UP_TWICE
# LOCAL_CFLAGS += -D_DMA_COPY_OSD_LAYER
#
# if GSP has not IOMMU, DIRECT_DISPLAY_SINGLE_OSD_LAYER need contiguous physcial address;
# if GSP has IOMMU, we can open DIRECT_DISPLAY_SINGLE_OSD_LAYER.
ifeq ($(strip $(DEVICE_DIRECT_DISPLAY_SINGLE_OSD_LAYER)),true)
LOCAL_CFLAGS += -DDIRECT_DISPLAY_SINGLE_OSD_LAYER
endif

ifeq ($(strip $(TARGET_BOARD_PLATFORM)),sc8830)
ifneq ($(strip $(DEVICE_GSP_NOT_SCALING_UP_TWICE)),true) # when on tshark, if cpy2_pa is exist, we dont support scaling-up-twice feature
endif
endif

ifeq ($(TARGET_BOARD_PLATFORM),scx15)
LOCAL_CFLAGS += -DGSP_ADDR_TYPE_PHY
endif

endif # DEVICE_WITH_GSP

ifeq ($(DEVICE_PRIMARYPLANE_USE_RGB565),true)
LOCAL_CFLAGS += -DPRIMARYPLANE_USE_RGB565
endif

ifeq ($(DEVICE_OVERLAYPLANE_BORROW_PRIMARYPLANE_BUFFER),true)
LOCAL_CFLAGS += -DBORROW_PRIMARYPLANE_BUFFER
endif

ifeq ($(DEVICE_DYNAMIC_RELEASE_PLANEBUFFER),true)
LOCAL_CFLAGS += -DDYNAMIC_RELEASE_PLANEBUFFER
endif

# For Virtual Display
# HWC need do the Hardware copy and format convertion
# FORCE_ADJUST_ACCELERATOR: for a better performance of Virtual Display,
# we forcibly make sure the GSP/GPP device to be used by Virtual Display.
# and disable the GSP/GPP on Primary Display.
ifeq ($(TARGET_FORCE_HWC_FOR_VIRTUAL_DISPLAYS),true)
LOCAL_CFLAGS += -DFORCE_HWC_COPY_FOR_VIRTUAL_DISPLAYS
#LOCAL_CFLAGS += -DFORCE_ADJUST_ACCELERATOR
endif

# OVERLAY_COMPOSER_GPU_CONFIG: Enable or disable OVERLAY_COMPOSER_GPU
# Macro, OVERLAY_COMPOSER will do Hardware layer blending and then
# post the overlay buffer to OSD display plane.
# If you want to know how OVERLAY_COMPOSER use and work,
# Please see the OverlayComposer/OverlayComposer.h for more details.
ifeq ($(strip $(USE_OVERLAY_COMPOSER_GPU)),true)

LOCAL_CFLAGS += \
	-DOVERLAY_COMPOSER_GPU

LOCAL_SRC_FILES += \
	OverlayComposer/OverlayComposer.cpp \
	OverlayComposer/OverlayNativeWindow.cpp \
	OverlayComposer/Layer.cpp \
	OverlayComposer/Utility.cpp \
	OverlayComposer/SyncThread.cpp \

endif

ifeq ($(TARGET_BOARD_PLATFORM),sc8825)

#LOCAL_CFLAGS += -DTRANSFORM_USE_GPU

LOCAL_CFLAGS += \
	-DSCAL_ROT_TMP_BUF \
	-D_PROC_OSD_WITH_THREAD \
	-D_DMA_COPY_OSD_LAYER

LOCAL_C_INCLUDES += $(LOCAL_PATH)/../libcamera/sc8825/inc

LOCAL_SRC_FILES += sc8825/scale_rotate.c

endif

ifeq ($(TARGET_BOARD_PLATFORM),sc8810)
LOCAL_CFLAGS += -DSCAL_ROT_TMP_BUF -D_VSYNC_USE_SOFT_TIMER
LOCAL_SRC_FILES += sc8810/scale_rotate.c

endif

ifeq ($(strip $(TARGET_BOARD_PLATFORM)),sc7710)
LOCAL_CFLAGS += -DSCAL_ROT_TMP_BUF -D_VSYNC_USE_SOFT_TIMER

LOCAL_SRC_FILES += sc8810/scale_rotate.c
endif

ifeq ($(USE_GPU_PROCESS_VIDEO),true)
LOCAL_CFLAGS += -DTRANSFORM_USE_GPU
LOCAL_SRC_FILES += gpu_transform.cpp
endif

ifeq ($(USE_RGB_VIDEO_LAYER),true)
LOCAL_CFLAGS += -DVIDEO_LAYER_USE_RGB
endif

ifeq ($(USE_SPRD_DITHER),true)
LOCAL_CFLAGS += -DSPRD_DITHER_ENABLE
endif

LOCAL_MODULE_TAGS := optional
include $(BUILD_SHARED_LIBRARY)

endif

include $(call all-makefiles-under,$(LOCAL_PATH))
