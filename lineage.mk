# Inherit from those products. Most specific first.
$(call inherit-product, $(SRC_TARGET_DIR)/product/full_base.mk)

# Inherit some common Lineage stuff.
$(call inherit-product, vendor/cm/config/common_full_tablet_wifionly.mk)

# Inherit from gtexswifi device
$(call inherit-product, device/samsung/gtexswifi/device.mk)

PRODUCT_DEVICE := gtexswifi
PRODUCT_NAME := lineage_gtexswifi
PRODUCT_BRAND := samsung
PRODUCT_MODEL := SM-T280
PRODUCT_MANUFACTURER := samsung

PRODUCT_BUILD_PROP_OVERRIDES += \
    BUILD_FINGERPRINT=samsung/gtexswifixx/gtexswifi:5.1.1/LMY47V/T280XXU0AQA4:user/release-keys \
    PRIVATE_BUILD_DESC="gtexswifixx-user 5.1.1 LMY47V T280XXU0AQA4 release-keys"
