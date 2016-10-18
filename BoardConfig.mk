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
BOARD_VENDOR := samsung

ARCH_ARM_HAVE_TLS_REGISTER := true
TARGET_BOOTLOADER_BOARD_NAME := gtexslte

BOARD_KERNEL_CMDLINE := console=ttyS1,115200n8
BOARD_KERNEL_BASE := 0x80000000
BOARD_KERNEL_PAGESIZE := 2048

# Audio
BOARD_USES_TINYALSA_AUDIO := true
BOARD_USES_SS_VOIP := true
BOARD_USE_LIBATCHANNEL_WRAPPER := true

# something
BOARD_USE_SAMSUNG_COLORFORMAT := true
BOARD_NEEDS_MEMORYHEAPION_SPRD := true
COMMON_GLOBAL_CFLAGS += -DSPRD_HARDWARE


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

USE_SPRD_HWCOMPOSER := true
USE_OPENGL_RENDERER := true
BOARD_USE_MHEAP_SCREENSHOT := true
TARGET_BOARD_PLATFORM_GPU := ARM Mali-400
USE_OVERLAY_COMPOSER_GPU := true
DEVICE_FORCE_VIDEO_GO_OVERLAYCOMPOSER := true
BOARD_EGL_WORKAROUND_BUG_10194508 := true
TARGET_RUNNING_WITHOUT_SYNC_FRAMEWORK := true
COMMON_GLOBAL_CFLAGS += -DSC8830_HWC

# TARGET_KERNEL_SOURCE := kernel/samsung/gtexslte
# TARGET_KERNEL_CONFIG := gtexslte_defconfig
# TARGET_VARIANT_CONFIG := gtexslte_defconfig
# TARGET_SELINUX_CONFIG := gtexslte_defconfig

# Bluetooth
BOARD_HAVE_BLUETOOTH := true
# BOARD_BLUETOOTH_BDROID_BUILDCFG_INCLUDE_DIR := device/samsung/gtexslte/bluetooth
# BOARD_BLUEDROID_VENDOR_CONF := device/samsung/gtexslte/bluetooth/libbt_vndcfg.txt

# Wifi
# BOARD_WLAN_DEVICE := sc2331
# BOARD_WLAN_DEVICE_REV := BCM4330B1_002.001.003.1025.1303
# WPA_SUPPLICANT_VERSION := VER_0_8_X
# BOARD_WPA_SUPPLICANT_DRIVER := NL80211
# BOARD_WPA_SUPPLICANT_PRIVATE_LIB := lib_driver_cmd_bcmdhd
# BOARD_HOSTAPD_DRIVER := NL80211
# BOARD_HOSTAPD_PRIVATE_LIB := lib_driver_cmd_bcmdhd
# WIFI_DRIVER_FW_PATH_PARAM := "/sys/module/dhd/parameters/firmware_path"
# WIFI_DRIVER_FW_PATH_STA := "/system/etc/wifi/bcmdhd_sta.bin"
# WIFI_DRIVER_FW_PATH_AP := "/system/etc/wifi/bcmdhd_apsta.bin"
# WIFI_DRIVER_NVRAM_PATH_PARAM := "/sys/module/dhd/parameters/nvram_path"
# WIFI_DRIVER_NVRAM_PATH := "/system/etc/wifi/nvram_net.txt"
# WIFI_BAND := 802_11_ABG
# BOARD_HAVE_SAMSUNG_WIFI := true


TARGET_PREBUILT_KERNEL := kernel/samsung/gtexslte/arch/arm/boot/zImage

# SC9830_MODULES:
# 	mkdir -p $(PRODUCT_OUT)/root/lib/modules
# 	mkdir -p $(PRODUCT_OUT)/recovery/root/lib/modules
# 	make -C $(TARGET_KERNEL_SOURCE)/external_module/mali MALI_PLATFORM=sc8830 BUILD=release KDIR=$(KERNEL_OUT)
# 	cp $(TARGET_KERNEL_SOURCE)/external_module/mali/*.ko $(PRODUCT_OUT)/root/lib/modules
# 	cp $(TARGET_KERNEL_SOURCE)/external_module/mali/*.ko $(PRODUCT_OUT)/recovery/root/lib/modules
# 	make -C $(TARGET_KERNEL_SOURCE)/external_module/wifi clean
# 	make -C $(TARGET_KERNEL_SOURCE)/external_module/wifi KDIR=$(KERNEL_OUT)
# 	cp $(TARGET_KERNEL_SOURCE)/external_module/wifi/*.ko $(PRODUCT_OUT)/root/lib/modules
# 	cp $(TARGET_KERNEL_SOURCE)/external_module/wifi/*.ko $(PRODUCT_OUT)/recovery/root/lib/modules
# 	find ${KERNEL_OUT}/drivers -name "*.ko" -exec cp -f {} $(PRODUCT_OUT)/root/lib/modules \;
#
# TARGET_KERNEL_MODULES := SC9830_MODULES

BOARD_SEPOLICY_DIRS += device/samsung/gtexslte/sepolicy

BOARD_SEPOLICY_UNION += \
			 bluetooth.te \
			 device.te \
			 batterysrv.te \
       debuggerd.te \
       init.te \
			 surfaceflinger.te \
			 rild.te \
			 systemserver.te \
			 netd.te \
			 mfgloader.te \
			 mediaserver.te \
			 system_app.te \
			 at_distributor.te \
			 download.te \
			 engpc.te \
			 cp_diskserver.te \
			 refnotify.te \
			 wlandutservice.te \
			 data_on_off.te \
			 macloader.te \
			 slogmodem.te \
			 smd_symlink.te \
			 ddexe.te \
			 connfwexe.te \
			 untrusted_app.te \
			 genfs_contexts \
			 vold.te \
			 prepare_param.te \
       file_contexts


TARGET_RECOVERY_FSTAB = device/samsung/gtexslte/recovery.fstab
RECOVERY_VARIANT := twrp
TARGET_RECOVERY_PIXEL_FORMAT := "BGRA_8888"
RECOVERY_GRAPHICS_USE_LINELENGTH := true
BOARD_HAS_NO_MISC_PARTITION := true
TW_THEME := portrait_hdpi
TW_HAS_DOWNLOAD_MODE := true
BOARD_HAS_FLIPPED_SCREEN := true
BOARD_HAS_NO_SELECT_BUTTON := true
