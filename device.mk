### device.mk ###

$(call inherit-product, $(SRC_TARGET_DIR)/product/languages_full.mk)

# Inherit from vendor tree
$(call inherit-product-if-exists, vendor/samsung/gtexswifi/gtexswifi-vendor.mk)

# Boot animation
TARGET_SCREEN_WIDTH := 800
TARGET_SCREEN_HEIGHT := 1280
TARGET_BOOTANIMATION_HALF_RES := true

# Overlays
DEVICE_PACKAGE_OVERLAYS += $(LOCAL_PATH)/overlay

# This device is hdpi
PRODUCT_AAPT_CONFIG := normal
PRODUCT_AAPT_PREF_CONFIG := hdpi

# Media
MEDIA_CONFIGS := \
    $(LOCAL_PATH)/configs/media/media_codecs.xml \
    $(LOCAL_PATH)/configs/media/media_profiles.xml \
    $(LOCAL_PATH)/configs/media/media_codecs_performance.xml \
    frameworks/av/media/libstagefright/data/media_codecs_google_audio.xml \
    frameworks/av/media/libstagefright/data/media_codecs_google_video.xml \
    frameworks/av/media/libstagefright/data/media_codecs_google_video_le.xml \
    frameworks/av/media/libstagefright/data/media_codecs_google_telephony.xml

PRODUCT_COPY_FILES += \
    $(foreach f,$(MEDIA_CONFIGS),$(f):system/etc/$(notdir $(f)))

# GPS
PRODUCT_COPY_FILES += \
    $(LOCAL_PATH)/configs/gps/gps.conf:system/etc/gps.conf \
    $(LOCAL_PATH)/configs/gps/gps.xml:system/etc/gps.xml \
    $(LOCAL_PATH)/configs/gps/gnss_cfg.xml:system/etc/gnss_cfg.xml

# System init .rc files
SYSTEM_INIT_RC_FILES := \
    $(LOCAL_PATH)/system/etc/init/batterysrv.rc \
    $(LOCAL_PATH)/system/etc/init/cp_diskserver.rc \
    $(LOCAL_PATH)/system/etc/init/engpc.rc \
    $(LOCAL_PATH)/system/etc/init/mediacodec.rc \
    $(LOCAL_PATH)/system/etc/init/mediaserver.rc \
    $(LOCAL_PATH)/system/etc/init/modem_control.rc \
    $(LOCAL_PATH)/system/etc/init/modemd.rc \
    $(LOCAL_PATH)/system/etc/init/phoneserver.rc \
    $(LOCAL_PATH)/system/etc/init/prepare_param.rc \
    $(LOCAL_PATH)/system/etc/init/refnotify.rc \
    $(LOCAL_PATH)/system/etc/init/smd_symlink.rc \
    $(LOCAL_PATH)/system/etc/init/surfaceflinger.rc \
    $(LOCAL_PATH)/system/etc/init/wcnd.rc


PRODUCT_COPY_FILES += \
    $(foreach f,$(SYSTEM_INIT_RC_FILES),$(f):system/etc/init/$(notdir $(f)))

# Ramdisk
PRODUCT_COPY_FILES += \
    $(LOCAL_PATH)/rootdir/init.board.rc:root/init.board.rc \
    $(LOCAL_PATH)/rootdir/init.sc8830.rc:root/init.sc8830.rc \
    $(LOCAL_PATH)/rootdir/init.sc8830.usb.rc:root/init.sc8830.usb.rc \
    $(LOCAL_PATH)/rootdir/fstab.sc8830:root/fstab.sc8830 \
    $(LOCAL_PATH)/rootdir/ueventd.sc8830.rc:root/ueventd.sc8830.rc \
    $(LOCAL_PATH)/rootdir/init.wifi.rc:root/init.wifi.rc \
    $(LOCAL_PATH)/rootdir/init.dhcp.rc:root/init.dhcp.rc

# Bluetooth
PRODUCT_COPY_FILES += \
    $(LOCAL_PATH)/configs/bluetooth/pskey_bt.txt:system/lib/modules/pskey_bt.txt \
    $(LOCAL_PATH)/configs/bluetooth/iop_bt.db:system/etc/bluetooth/iop_bt.db \
    $(LOCAL_PATH)/configs/bluetooth/iop_device_list.conf:system/etc/bluetooth/iop_device_list.conf

# Prebuilt kl keymaps
PRODUCT_COPY_FILES += \
    $(LOCAL_PATH)/keylayout/sec_touchkey.kl:system/usr/keylayout/sec_touchkey.kl \
    $(LOCAL_PATH)/keylayout/sci-keypad.kl:system/usr/keylayout/sci-keypad.kl

#Wallpaper/Apps
PRODUCT_PACKAGES += \
    CMWallpapers

# OMS
PRODUCT_PACKAGES += \
    ThemeInterfacer

# Charger
PRODUCT_PACKAGES += \
    charger \
    charger_res_images

# Codecs
PRODUCT_PACKAGES += \
    libcolorformat_switcher \
    libstagefrighthw \
    libstagefright_sprd_mpeg4dec \
    libstagefright_sprd_mpeg4enc \
    libstagefright_sprd_h264dec \
    libstagefright_sprd_h264enc \
    libstagefright_sprd_vpxdec \
    libstagefright_sprd_aacdec \
    libstagefright_sprd_mp3dec \
    libstagefright_sprd_soft_mpeg4dec \
    libstagefright_sprd_soft_h264dec \
    libstagefright_soft_mjpgdec \
    libstagefright_soft_imaadpcmdec

#vpu
PRODUCT_PACKAGES += \
    libstagefrighthw_cm \
    libomxil-bellagio \
    libtheoraparser \
    libvpu \
    libomxvpu

# Graphics & HWC
PRODUCT_PACKAGES += \
    libHWCUtils \
    libGLES_mali.so \
    memtrack.sc8830 \
    gralloc.sc8830 \
    libdither \
    hwcomposer.sc8830 \
    sprd_gsp.sc8830 \
    libmemoryheapion \
    libion_sprd \
    libgps_shim \
    libstagefright_shim

PRODUCT_PACKAGES += \
    libhealthd.sc8830 \
    power.sc8830

# Usb accessory
PRODUCT_PACKAGES += \
    com.android.future.usb.accessory

# Bluetooth
PRODUCT_PACKAGES += \
    bluetooth.default \
    audio.a2dp.default \
    libbt-vendor \
    libbluetooth_jni

# Lights
PRODUCT_PACKAGES += \
    lights.sc8830

PRODUCT_PACKAGES += \
    libsprd_agps_agent

#trustzone
PRODUCT_PACKAGES += \
    libtrusty

# Audio
PRODUCT_PACKAGES += \
    audio.primary.sc8830 \
    audio_policy.sc8830 \
    libaudio-resampler \
    audio.a2dp.default \
    audio.usb.default \
    audio.r_submix.default \
    libtinyalsa \
    audio_hw.xml \
    audio_para \
    audio_effects.conf \
    audio_policy.conf \
    codec_pga.xml \
    tiny_hw.xml \
    libeng-audio \
    libatchannel

# Camera HAL
#PRODUCT_PACKAGES += \
#    camera.sc8830

# Wifi
PRODUCT_PACKAGES += \
    wpa_supplicant \
    hostapd

# Permissions
PERMISSION_XML_FILES := \
    frameworks/native/data/etc/tablet_core_hardware.xml \
    frameworks/native/data/etc/android.hardware.camera.autofocus.xml \
    frameworks/native/data/etc/android.hardware.camera.front.xml \
    frameworks/native/data/etc/android.hardware.camera.xml \
    frameworks/native/data/etc/android.hardware.bluetooth.xml \
    frameworks/native/data/etc/android.hardware.bluetooth_le.xml \
    frameworks/native/data/etc/android.hardware.location.gps.xml \
    frameworks/native/data/etc/android.hardware.sensor.accelerometer.xml \
    frameworks/native/data/etc/android.hardware.touchscreen.multitouch.xml \
    frameworks/native/data/etc/android.hardware.touchscreen.multitouch.jazzhand.xml \
    frameworks/native/data/etc/android.hardware.touchscreen.xml \
    frameworks/native/data/etc/android.hardware.telephony.gsm.xml \
    frameworks/native/data/etc/android.hardware.usb.accessory.xml \
    frameworks/native/data/etc/android.hardware.usb.host.xml \
    frameworks/native/data/etc/android.software.sip.voip.xml \
    frameworks/native/data/etc/android.software.sip.xml \
    frameworks/native/data/etc/android.hardware.wifi.xml \
    frameworks/native/data/etc/android.hardware.wifi.direct.xml \
    frameworks/native/data/etc/android.software.midi.xml

PRODUCT_COPY_FILES += \
    $(foreach f,$(PERMISSION_XML_FILES),$(f):system/etc/permissions/$(notdir $(f)))

# Dalvik Heap config
include frameworks/native/build/tablet-7in-hdpi-1024-dalvik-heap.mk

