LOCAL_PATH := device/samsung/gtexswifi

TARGET_OTA_ASSERT_DEVICE := SM-T280,gtexswifi

# Architecture
TARGET_ARCH := arm
TARGET_ARCH_VARIANT := armv7-a-neon
TARGET_CPU_ABI := armeabi-v7a
TARGET_CPU_ABI2 := armeabi
TARGET_CPU_VARIANT := cortex-a7
TARGET_CPU_SMP := true

# Bootloader
TARGET_BOOTLOADER_BOARD_NAME := SC7730SW

# Kernel
BOARD_KERNEL_BASE := 0x00000000
BOARD_KERNEL_CMDLINE := console=ttyS1,115200n8
BOARD_KERNEL_PAGESIZE := 2048
BOARD_MKBOOTIMG_ARGS := --kernel_offset 0x00008000 --ramdisk_offset 0x01000000 --tags_offset 0x00000100 --dt device/samsung/gtexswifi/dt.img
TARGET_KERNEL_CONFIG := lineageos_gtexswifi_defconfig
TARGET_KERNEL_SELINUX_CONFIG := lineageos_gtexswifi_defconfig
TARGET_VARIANT_CONFIG := lineageos_gtexswifi_defconfig
TARGET_KERNEL_SOURCE := kernel/samsung/gtexswifi

# Partitions
BOARD_BOOTIMAGE_PARTITION_SIZE     := 16777216
BOARD_RECOVERYIMAGE_PARTITION_SIZE := 16777216
BOARD_SYSTEMIMAGE_PARTITION_SIZE   := 2147483648
BOARD_USERDATAIMAGE_PARTITION_SIZE := 5289017344
BOARD_CACHEIMAGE_PARTITION_SIZE    := 209715200
BOARD_FLASH_BLOCK_SIZE := 131072

# Platform
TARGET_BOARD_PLATFORM := sc8830

# Recovery
TARGET_USERIMAGES_USE_EXT4 := true
TARGET_USERIMAGES_USE_F2FS := true

# Sepolicy
BOARD_SEPOLICY_DIRS += \
    device/samsung/gtexswifi/sepolicy

# inherit from the proprietary version
-include vendor/samsung/gtexswifi/BoardConfigVendor.mk

