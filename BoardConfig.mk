USE_CAMERA_STUB := true

# inherit from the proprietary version
-include vendor/samsung/gtexslte/BoardConfigVendor.mk

TARGET_ARCH := arm
TARGET_NO_BOOTLOADER := false
TARGET_NO_RADIOIMAGE := true

TARGET_BOARD_PLATFORM := sc8830
TARGET_CPU_ABI := armeabi-v7a
TARGET_CPU_ABI2 := armeabi
TARGET_ARCH_VARIANT := armv7-a-neon
TARGET_CPU_VARIANT := cortex-a7
TARGET_CPU_SMP := true
ARCH_ARM_HAVE_TLS_REGISTER := true

TARGET_BOOTLOADER_BOARD_NAME := gtexslte

BOARD_KERNEL_CMDLINE := console=ttyS1,115200n8
BOARD_KERNEL_BASE := 0x80000000
BOARD_KERNEL_PAGESIZE := 2048

# fix this up by examining /proc/mtd on a running device
TARGET_USERIMAGES_USE_EXT4 := true
TARGET_USERIMAGES_USE_F2FS := true
BOARD_BOOTIMAGE_PARTITION_SIZE := 16777216
BOARD_RECOVERYIMAGE_PARTITION_SIZE := 16777216
BOARD_SYSTEMIMAGE_PARTITION_SIZE := 1932734976
BOARD_USERDATAIMAGE_PARTITION_SIZE := 5495377408
BOARD_CACHEIMAGE_PARTITION_SIZE := 209715200
BOARD_CACHEIMAGE_FILE_SYSTEM_TYPE := ext4
BOARD_FLASH_BLOCK_SIZE := 131072

USE_OPENGL_RENDERER := true
TARGET_USES_ION := true

BOARD_EGL_CFG := device/samsung/gtexslte/egl.cfg

TARGET_KERNEL_SOURCE := kernel/samsung/gtexslte
TARGET_KERNEL_CONFIG := gtexslte_defconfig

# TARGET_PREBUILT_KERNEL := device/samsung/gtexslte/kernel
SC9830_MODULES:
	mkdir -p $(PRODUCT_OUT)/root/lib/modules
	mkdir -p $(PRODUCT_OUT)/recovery/root/lib/modules
	make -C $(TARGET_KERNEL_SOURCE)/external_module/mali MALI_PLATFORM=sc8830 BUILD=release KDIR=$(KERNEL_OUT)
	cp $(TARGET_KERNEL_SOURCE)/external_module/mali/*.ko $(PRODUCT_OUT)/root/lib/modules
	cp $(TARGET_KERNEL_SOURCE)/external_module/mali/*.ko $(PRODUCT_OUT)/recovery/root/lib/modules
	make -C $(TARGET_KERNEL_SOURCE)/external_module/wifi KDIR=$(KERNEL_OUT)
	cp $(TARGET_KERNEL_SOURCE)/external_module/wifi/*.ko $(PRODUCT_OUT)/root/lib/modules
	cp $(TARGET_KERNEL_SOURCE)/external_module/wifi/*.ko $(PRODUCT_OUT)/recovery/root/lib/modules

TARGET_KERNEL_MODULES := SC9830_MODULES

TARGET_RECOVERY_FSTAB = device/samsung/gtexslte/recovery.fstab
RECOVERY_VARIANT := twrp
TW_THEME := portrait_hdpi
BOARD_HAS_FLIPPED_SCREEN := true

BOARD_HAS_NO_SELECT_BUTTON := true
