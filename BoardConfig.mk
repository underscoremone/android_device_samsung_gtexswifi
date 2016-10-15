USE_CAMERA_STUB := true

# inherit from the proprietary version
-include vendor/samsung/gtexslte/BoardConfigVendor.mk

TARGET_ARCH := arm
TARGET_NO_BOOTLOADER := false
TARGET_NO_RADIOIMAGE := true
TARGET_USES_ION := true
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

TARGET_USERIMAGES_USE_EXT4 := true
BOARD_BOOTIMAGE_PARTITION_SIZE := 16777216
BOARD_RECOVERYIMAGE_PARTITION_SIZE := 16777216
BOARD_SYSTEMIMAGE_PARTITION_SIZE := 1932734976
BOARD_USERDATAIMAGE_PARTITION_SIZE := 5495377408
BOARD_CACHEIMAGE_PARTITION_SIZE := 209715200
BOARD_CACHEIMAGE_FILE_SYSTEM_TYPE := ext4
BOARD_FLASH_BLOCK_SIZE := 131072

USE_OPENGL_RENDERER := true

TARGET_BOARD_INFO_FILE := device/samsung/gtexslte/board-info.txt
BOARD_EGL_CFG := device/samsung/gtexslte/egl.cfg

USE_OPENGL_RENDERER := true
BOARD_USE_MHEAP_SCREENSHOT := true
TARGET_BOARD_PLATFORM_GPU := ARM Mali-400
BOARD_EGL_WORKAROUND_BUG_10194508 := true
TARGET_RUNNING_WITHOUT_SYNC_FRAMEWORK := true

# TARGET_KERNEL_SOURCE := kernel/samsung/gtexslte
# TARGET_KERNEL_CONFIG := gtexslte_defconfig
# TARGET_VARIANT_CONFIG := gtexslte_defconfig
# TARGET_SELINUX_CONFIG := gtexslte_defconfig

TARGET_PREBUILT_KERNEL := device/samsung/gtexslte/kernel

SC9830_MODULES:
	mkdir -p $(PRODUCT_OUT)/root/lib/modules
	mkdir -p $(PRODUCT_OUT)/recovery/root/lib/modules
	make -C $(TARGET_KERNEL_SOURCE)/external_module/mali MALI_PLATFORM=sc8830 BUILD=release KDIR=$(KERNEL_OUT)
	cp $(TARGET_KERNEL_SOURCE)/external_module/mali/*.ko $(PRODUCT_OUT)/root/lib/modules
	cp $(TARGET_KERNEL_SOURCE)/external_module/mali/*.ko $(PRODUCT_OUT)/recovery/root/lib/modules
	make -C $(TARGET_KERNEL_SOURCE)/external_module/wifi clean
	make -C $(TARGET_KERNEL_SOURCE)/external_module/wifi KDIR=$(KERNEL_OUT)
	cp $(TARGET_KERNEL_SOURCE)/external_module/wifi/*.ko $(PRODUCT_OUT)/root/lib/modules
	cp $(TARGET_KERNEL_SOURCE)/external_module/wifi/*.ko $(PRODUCT_OUT)/recovery/root/lib/modules
	find ${KERNEL_OUT}/drivers -name "*.ko" -exec cp -f {} $(PRODUCT_OUT)/root/lib/modules \;

TARGET_KERNEL_MODULES := SC9830_MODULES

BOARD_SEPOLICY_DIRS += device/samsung/gtexslte/sepolicy

BOARD_SEPOLICY_UNION += \
       debuggerd.te \
       init.te \
			 surfaceflinger.te \
			 rild.te \
			 systemserver.te \
       file_contexts \
			 netd.te \
			 mediaserver.te \
			 system_app.te


TARGET_RECOVERY_FSTAB = device/samsung/gtexslte/recovery.fstab
RECOVERY_VARIANT := twrp
TARGET_RECOVERY_PIXEL_FORMAT := "RGBX_8888"
RECOVERY_GRAPHICS_USE_LINELENGTH := true
BOARD_HAS_NO_MISC_PARTITION := true
TW_THEME := portrait_hdpi
TW_HAS_DOWNLOAD_MODE := true
BOARD_HAS_FLIPPED_SCREEN := true

BOARD_HAS_NO_SELECT_BUTTON := true
