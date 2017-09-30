$(call inherit-product, $(SRC_TARGET_DIR)/product/languages_full.mk)

# The gps config appropriate for this device
# $(call inherit-product, device/common/gps/gps_as_supl.mk)

$(call inherit-product-if-exists, vendor/samsung/gtexslte/gtexslte-vendor.mk)

DEVICE_PACKAGE_OVERLAYS += device/samsung/gtexslte/overlay

# This device is hdpi
PRODUCT_AAPT_CONFIG := normal
PRODUCT_AAPT_PREF_CONFIG := hdpi

ifeq ($(TARGET_PREBUILT_KERNEL),)
	LOCAL_KERNEL := device/samsung/gtexslte/kernel
else
	LOCAL_KERNEL := $(TARGET_PREBUILT_KERNEL)
endif

# Media config
MEDIA_CONFIGS := \
	$(LOCAL_PATH)/media/media_codecs.xml \
	$(LOCAL_PATH)/media/media_profiles.xml \
	frameworks/av/media/libstagefright/data/media_codecs_google_audio.xml \
	frameworks/av/media/libstagefright/data/media_codecs_google_video.xml \
	frameworks/av/media/libstagefright/data/media_codecs_google_telephony.xml

PRODUCT_COPY_FILES += \
	$(foreach f,$(MEDIA_CONFIGS),$(f):system/etc/$(notdir $(f)))

# System init .rc files
SYSTEM_INIT_RC_FILES := \
		device/samsung/gtexslte/system/etc/init/at_distributor.rc \
		device/samsung/gtexslte/system/etc/init/engpc.rc \
		device/samsung/gtexslte/system/etc/init/mediacodec.rc \
		device/samsung/gtexslte/system/etc/init/mediaserver.rc \
		device/samsung/gtexslte/system/etc/init/rild.rc \
		device/samsung/gtexslte/system/etc/init/surfaceflinger.rc \

PRODUCT_COPY_FILES += \
	$(foreach f,$(SYSTEM_INIT_RC_FILES),$(f):system/etc/init/$(notdir $(f)))

PRODUCT_COPY_FILES += \
		device/samsung/gtexslte/init.board.rc:root/init.board.rc \
		device/samsung/gtexslte/init.sc8830.rc:root/init.sc8830.rc \
		device/samsung/gtexslte/init.sc8830.usb.rc:root/init.sc8830.usb.rc \
		device/samsung/gtexslte/init.gtexslte.rc:root/init.gtexslte.rc \
		device/samsung/gtexslte/init.gtexslte_base.rc:root/init.gtexslte_base.rc \
    device/samsung/gtexslte/init.sc8830_ss.rc:root/init.sc8830_ss.rc \
    device/samsung/gtexslte/init.gtexslte.usb.rc:root/init.gtexslte.usb.rc \
    device/samsung/gtexslte/fstab.sc8830:root/fstab.sc8830 \
		device/samsung/gtexslte/fstab.sc8830:root/fstab.unknown \
    device/samsung/gtexslte/ueventd.sc8830.rc:root/ueventd.sc8830.rc \
		device/samsung/gtexslte/ueventd.sc8830.rc:root/ueventd.unknown.rc \
		device/samsung/gtexslte/init.rilcommon.rc:root/init.rilcommon.rc \
		device/samsung/gtexslte/init.wifi.rc:root/init.wifi.rc \
		device/samsung/gtexslte/init.dhcp.rc:root/init.dhcp.rc \
		device/samsung/gtexslte/apns-conf.xml:system/etc/apns-conf.xml \
		device/samsung/gtexslte/bluetooth:system/etc/bluetooth \

PRODUCT_COPY_FILES += \
	device/samsung/gtexslte/audio_policy_config/audio_policy_configuration.xml:system/etc/audio_policy_configuration.xml \
	device/samsung/gtexslte/audio_policy_config/audio_policy_configuration_stub.xml:system/etc/audio_policy_configuration_stub.xml \
	device/samsung/gtexslte/audio_policy_config/a2dp_audio_policy_configuration.xml:system/etc/a2dp_audio_policy_configuration.xml \
	device/samsung/gtexslte/audio_policy_config/usb_audio_policy_configuration.xml:system/etc/usb_audio_policy_configuration.xml \
	device/samsung/gtexslte/audio_policy_config/r_submix_audio_policy_configuration.xml:system/etc/r_submix_audio_policy_configuration.xml \
	device/samsung/gtexslte/audio_policy_config/audio_policy_volumes.xml:system/etc/audio_policy_volumes.xml \
	device/samsung/gtexslte/audio_policy_config/default_volume_tables.xml:system/etc/default_volume_tables.xml

# gps
PRODUCT_COPY_FILES += \
	device/samsung/gtexslte/gps/gps.conf:system/etc/gps.conf \
	device/samsung/gtexslte/gps/gps.xml:system/etc/gps.xml \

# audio configs
PRODUCT_COPY_FILES += \
	device/samsung/gtexslte/audio/audio_hw.xml:system/etc/audio_hw.xml \
	device/samsung/gtexslte/audio/audio_para:system/etc/audio_para \
	device/samsung/gtexslte/audio/codec_pga.xml:system/etc/codec_pga.xml \
	device/samsung/gtexslte/audio/tiny_hw.xml:system/etc/tiny_hw.xml \

$(call inherit-product, build/target/product/full.mk)

PRODUCT_BUILD_PROP_OVERRIDES += BUILD_UTC_DATE=0

# Prebuilt kl keymaps
PRODUCT_COPY_FILES += \
	$(LOCAL_PATH)/keylayout/sec_touchkey.kl:system/usr/keylayout/sec_touchkey.kl \
	$(LOCAL_PATH)/keylayout/sci-keypad.kl:system/usr/keylayout/sci-keypad.kl

#Wallpaper/Apps
PRODUCT_PACKAGES += \
	CMWallpapers \

# OMS
PRODUCT_PACKAGES += \
	ThemeInterfacer \

# Charger
PRODUCT_PACKAGES += \
	charger \
	charger_res_images

# Codecs
PRODUCT_PACKAGES += \
	libstagefright_shim \
	libcolorformat_switcher \
	libstagefrighthw \
	libstagefright_sprd_soft_mpeg4dec \
	libstagefright_sprd_soft_h264dec \
	libstagefright_sprd_mpeg4dec \
	libstagefright_sprd_mpeg4enc \
	libstagefright_sprd_h264dec \
	libstagefright_sprd_h264enc \
	libstagefright_sprd_vpxdec \
	libstagefright_soft_mjpgdec \
	libstagefright_soft_imaadpcmdec \
	libstagefright_sprd_mp3dec

#vpu
PRODUCT_PACKAGES += \
	libstagefrighthw_cm \
	libomxil-bellagio \
	libtheoraparser \
	libvpu \
	libomxvpu

PRODUCT_PROPERTY_OVERRIDES += \
		persist.ttydev=ttyVUART0 \
		ro.sf.lcd_density=213 \
		ro.sf.hwrotation=180 \
		ro.sf.lcd_width=96 \
		ro.sf.lcd_height=150 \
		ro.opengles.version=131072 \
		ro.product.hardware=SS_SHARKLS \
		ro.product.modem.mode=GSM,EDGE,TD-SCDMA,WCDMA,TD-LTE,FDD-LTE \
		ro.product.partitionpath=/dev/block/platform/sdio_emmc/by-name/ \
		ro.adb.secure=0 \
		ro.secure=0 \
		persist.sys.usb.config=mtp,adb \
		ro.cmlegal.url=https://lineageos.org/legal \
		ro.lineageoms.version=$(LINEAGE_VERSION)


PRODUCT_DEFAULT_PROPERTY_OVERRIDES += \
		ro.adb.secure=0 \
		ro.secure=0 \
		persist.sys.usb.config=mtp,adb \
		telephony.lteOnCdmaDevice=0 \
		persist.radio.apm_sim_not_pwdn=1 \
		persist.radio.add_power_save=1 \
		rild.libpath=/system/lib/libsec-ril.so

# Graphics & HWC
PRODUCT_PACKAGES += \
		libHWCUtils \
		libGLES_mali.so \
		memtrack.sc8830 \
		gralloc.sc8830.so \
		libdither \
		hwcomposer.sc8830 \
		sprd_gsp.sc8830 \
		libmemoryheapion \
		libion_sprd \
		libstagefright_shim \
		libgps_shim

PRODUCT_PACKAGES += \
		 libhealthd.sc8830 \
		 power.sc8830 \

# Usb accessory
PRODUCT_PACKAGES += \
	com.android.future.usb.accessory

# Bluetooth
PRODUCT_PACKAGES += \
	bluetooth.default \
	audio.a2dp.default \
	libbt-vendor

# Bluetooth
PRODUCT_PACKAGES += \
	libbluetooth_jni \


# Lights
PRODUCT_PACKAGES += \
	lights.sc8830

PRODUCT_PACKAGES += \
	libsprd_agps_agent

#trustzone
PRODUCT_PACKAGES += \
	libtrusty

PRODUCT_COPY_FILES += \
	$(foreach f,$(MEDIA_CONFIGS),$(f):system/etc/$(notdir $(f)))

# Audio
PRODUCT_PACKAGES += \
	audio.primary.sc8830 \
	audio_policy.sc8830 \
	audio.r_submix.default \
	audio.usb.default \
	audio_vbc_eq \
	libatchannel_wrapper \
	libaudio-resampler \
	libtinyalsa \
	libeng-audio \

# Camera HAL
PRODUCT_PACKAGES += \
	camera.sc8830

# Wifi
PRODUCT_PACKAGES += \
	wpa_supplicant \
	hostapd \

#ril
PRODUCT_PACKAGES += \
	engpc \
	wcnd \
	wcnd_cli \
	libril_shim \

PRODUCT_PACKAGES += \
	degas-mkbootimg

# Permissions
PERMISSION_XML_FILES := \
	frameworks/native/data/etc/handheld_core_hardware.xml \
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

	# enable Google-specific location features,
	# like NetworkLocationProvider and LocationCollector
PRODUCT_PROPERTY_OVERRIDES += \
	ro.com.google.locationfeatures=1 \
	ro.com.google.networklocation=1 \
	ro.sys.sdcardfs=true \

# Dalvik Heap config
include frameworks/native/build/tablet-7in-hdpi-1024-dalvik-heap.mk

PRODUCT_NAME := full_gtexslte
PRODUCT_DEVICE := gtexslte
