# Release name
PRODUCT_RELEASE_NAME := gtexslte

# Inherit some common CM stuff.
$(call inherit-product, vendor/cm/config/common_full_phone.mk)

# Inherit device configuration
$(call inherit-product, device/samsung/gtexslte/device_gtexslte.mk)

## Device identifier. This must come after all inclusions
PRODUCT_DEVICE := gtexslte
PRODUCT_NAME := cm_gtexslte
PRODUCT_BRAND := samsung
PRODUCT_MODEL := gtexslte
PRODUCT_MANUFACTURER := samsung
