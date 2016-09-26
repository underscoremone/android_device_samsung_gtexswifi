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
		device/samsung/gtexslte/init.sc8830.rc:root/init.gtexslte.rc \
    device/samsung/gtexslte/init.sc8830_ss.rc:root/init.sc8830_ss.rc \
    device/samsung/gtexslte/init.sc8830.usb.rc:root/init.sc8830.usb.rc \
    device/samsung/gtexslte/fstab.gtexslte:root/fstab.gtexslte \
    device/samsung/gtexslte/ueventd.sc8830.rc:root/ueventd.gtexslte.rc \
		device/samsung/gtexslte/init.wifi.rc:root/init.wifi.rc \
		device/samsung/gtexslte/init.dhcp.rc:root/init.dhcp.rc

$(call inherit-product, build/target/product/full.mk)

PRODUCT_BUILD_PROP_OVERRIDES += BUILD_UTC_DATE=0
PRODUCT_NAME := full_gtexslte
PRODUCT_DEVICE := gtexslte
