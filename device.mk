LOCAL_PATH := device/samsung/gtexswifi

$(call inherit-product, $(SRC_TARGET_DIR)/product/languages_full.mk)

# Get non-open-source specific aspects
$(call inherit-product-if-exists, vendor/samsung/gtexswifi/gtexswifi-vendor.mk)

# Overlays
DEVICE_PACKAGE_OVERLAYS += $(LOCAL_PATH)/overlay

PRODUCT_AAPT_CONFIG := normal
PRODUCT_AAPT_PREF_CONFIG := hdpi

$(call inherit-product, frameworks/native/build/tablet-7in-hdpi-1024-dalvik-heap.mk)
$(call inherit-product-if-exists, frameworks/native/build/tablet-7in-hdpi-1024-hwui-memory.mk)

# Boot animation
TARGET_SCREEN_WIDTH := 800
TARGET_SCREEN_HEIGHT := 1280
TARGET_BOOTANIMATION_HALF_RES := true

# Audio
PRODUCT_PACKAGES += \
    audio_effects.conf \
    audio_hw.xml \
    audio_para \
    audio_policy.conf \
    codec_pga.xml \
    tiny_hw.xml

# GPS
PRODUCT_PACKAGES += \
    gps.xml

# Keylayouts
PRODUCT_COPY_FILES += \
    $(LOCAL_PATH)/keylayout/sci-keypad.kl:system/usr/keylayout/sci-keypad.kl \
    $(LOCAL_PATH)/keylayout/sec_touchkey.kl:system/usr/keylayout/sec_touchkey.kl

# Ramdisk
PRODUCT_PACKAGES += \
    fstab.sc8830 \
    init.board.rc \
    init.sc8830.rc \
    init.sc8830.usb.rc \
    init.storage.rc \
    init.wifi.rc \
    ueventd.sc8830.rc

# WiFi
PRODUCT_PACKAGES += \
    p2p_supplicant_overlay.conf \
    wpa_supplicant.conf \
    wpa_supplicant_overlay.conf

