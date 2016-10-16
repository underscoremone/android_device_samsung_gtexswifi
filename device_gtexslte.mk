$(call inherit-product, $(SRC_TARGET_DIR)/product/languages_full.mk)

# The gps config appropriate for this device
$(call inherit-product, device/common/gps/gps_us_supl.mk)

$(call inherit-product-if-exists, vendor/samsung/gtexslte/gtexslte-vendor.mk)

DEVICE_PACKAGE_OVERLAYS += device/samsung/gtexslte/overlay


ifeq ($(TARGET_PREBUILT_KERNEL),)
	LOCAL_KERNEL := device/samsung/gtexslte/kernel
else
	LOCAL_KERNEL := $(TARGET_PREBUILT_KERNEL)
endif

PRODUCT_COPY_FILES += \
    $(LOCAL_KERNEL):kernel

PRODUCT_COPY_FILES += \
		device/samsung/gtexslte/init.board.rc:root/init.board.rc \
		device/samsung/gtexslte/init.sc8830.rc:root/init.sc8830.rc \
		device/samsung/gtexslte/init.gtexslte_base.rc:root/init.gtexslte_base.rc \
    device/samsung/gtexslte/init.sc8830_ss.rc:root/init.sc8830_ss.rc \
    device/samsung/gtexslte/init.gtexslte.usb.rc:root/init.gtexslte.usb.rc \
    device/samsung/gtexslte/fstab.sc8830:root/fstab.sc8830 \
    device/samsung/gtexslte/ueventd.sc8830.rc:root/ueventd.sc8830.rc \
		device/samsung/gtexslte/init.wifi.rc:root/init.wifi.rc \
		device/samsung/gtexslte/init.dhcp.rc:root/init.dhcp.rc

$(call inherit-product, build/target/product/full.mk)

PRODUCT_BUILD_PROP_OVERRIDES += BUILD_UTC_DATE=0

PRODUCT_PROPERTY_OVERRIDES += \
		persist.ttydev=ttyVUART0 \
		ro.sf.lcd_density=213 \
		ro.sf.hwrotation=180 \
		ro.opengles.version=131072 \
		ro.product.hardware=SS_SHARKLS \
		ro.product.modem.mode=GSM,EDGE,TD-SCDMA,WCDMA,TD-LTE,FDD-LTE \
		ro.product.partitionpath=/dev/block/platform/sdio_emmc/by-name/ \
		ro.adb.secure=0 \
		ro.secure=0 \
		persist.sys.usb.config=mtp,adb

PRODUCT_DEFAULT_PROPERTY_OVERRIDES += \
		ro.adb.secure=0 \
		ro.secure=0 \
		persist.sys.usb.config=mtp,adb \
		telephony.lteOnCdmaDevice=0 \
		persist.radio.apm_sim_not_pwdn=1 \
		persist.radio.add_power_save=1 \
		rild.libpath=/system/lib/libsec-ril.so

# HWC
PRODUCT_PACKAGES += \
		gralloc.sc8830 \
		hwcomposer.sc8830 \
		sprd_gsp.sc8830 \
		libion

PRODUCT_NAME := full_gtexslte
PRODUCT_DEVICE := gtexslte
