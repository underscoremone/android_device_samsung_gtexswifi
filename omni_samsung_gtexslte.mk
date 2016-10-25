# Release name
PRODUCT_RELEASE_NAME := gtexslte

$(call inherit-product, $(SRC_TARGET_DIR)/product/full_base_telephony.mk)

#---------------------------#
# Use generic Omni configs #
#---------------------------#
$(call inherit-product, vendor/omni/config/common.mk)


#---------------------------#
# Use generic Omni configs #
#---------------------------#
$(call inherit-product, device/samsung/gtexslte/device_gtexslte.mk)

# Use specific resolution for bootanimation
TARGET_BOOTANIMATION_SIZE := 720x480

# Exclude live wallpapers
TARGET_EXCLUDE_LIVEWALLPAPERS := true

## Device identifier. This must come after all inclusions
PRODUCT_DEVICE := gtexslte
PRODUCT_NAME := omni_samsung_gtexslte
PRODUCT_BRAND := samsung
PRODUCT_MODEL := SM-T285
PRODUCT_MANUFACTURER := samsung

TARGET_SCREEN_WIDTH := 800
TARGET_SCREEN_HEIGHT := 1280
