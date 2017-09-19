	USE_CAMERA_STUB := true

LOCAL_PATH := device/samsung/gtexslte
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
TARGET_BOOTLOADER_BOARD_NAME := 8830

BOARD_KERNEL_CMDLINE := console=ttyS1,115200n8
BOARD_KERNEL_BASE := 0
BOARD_KERNEL_PAGESIZE := 2048

#rild
BOARD_PROVIDES_RILD := true
BOARD_PROVIDES_LIBRIL := true
BOARD_PROVIDES_LIBREFERENCE_RIL := true

# Audio
USE_XML_AUDIO_POLICY_CONF := 1
SPRD_AUDIO_USE_NEW_API := true
USE_CUSTOM_AUDIO_POLICY := 1
BOARD_USES_TINYALSA_AUDIO := true
BOARD_USES_SS_VOIP := true
BOARD_USE_LIBATCHANNEL_WRAPPER := true
SOC_SCX30G_V2 := true

# something
# BOARD_USE_SAMSUNG_COLORFORMAT := true
BOARD_NEEDS_MEMORYHEAPION_SPRD := true
TARGET_SPRD_HARDWARE := true
TARGET_POWERHAL_VARIANT := samsung


BOARD_BOOTIMAGE_PARTITION_SIZE := 16777216
BOARD_RECOVERYIMAGE_PARTITION_SIZE := 16777216
BOARD_SYSTEMIMAGE_PARTITION_SIZE := 1932734976
BOARD_USERDATAIMAGE_PARTITION_SIZE := 5495377408
BOARD_CACHEIMAGE_PARTITION_SIZE := 209715200
BOARD_CACHEIMAGE_FILE_SYSTEM_TYPE := ext4
BOARD_FLASH_BLOCK_SIZE := 131072
TARGET_USERIMAGES_USE_EXT4 := true
TARGET_USERIMAGES_USE_F2FS := true
BOARD_HAS_LARGE_FILESYSTEM := true

TARGET_BOARD_INFO_FILE := device/samsung/gtexslte/board-info.txt
BOARD_EGL_CFG := device/samsung/gtexslte/egl.cfg

# Graphics
TARGET_RUNNING_WITHOUT_SYNC_FRAMEWORK := true
BOARD_EGL_NEEDS_HANDLE_VALUE := true
TARGET_REQUIRES_SYNCHRONOUS_SETSURFACE := true
TARGET_FORCE_SCREENSHOT_CPU_PATH := true
NUM_FRAMEBUFFER_SURFACE_BUFFERS := 3
TARGET_GPU_USE_TILE_ALIGN := true
USE_OVERLAY_COMPOSER_GPU := true
USE_UI_OVERLAY := true
USE_SPRD_DITHER := true
TARGET_GPU_PLATFORM := utgard
TARGET_USE_PREBUILT_GRALLOC := false
SOC_SCX30G_V2 := true

#1080 hw acceleration
BOARD_VSP_SUPPORT_1080I := true

# HWComposer
USE_SPRD_HWCOMPOSER := true
USE_OVERLAY_COMPOSER_GPU := true
TARGET_FORCE_HWC_FOR_VIRTUAL_DISPLAYS := true

# COMMON_GLOBAL_CFLAGS += -DSC8830_HWC
# COMMON_GLOBAL_CFLAGS += -DBOARD_CANT_REALLOCATE_OMX_BUFFERS

BOARD_RIL_CLASS := ../../../device/samsung/gtexslte/ril
BOARD_GLOBAL_CFLAGS += -DDISABLE_ASHMEM_TRACKING

# Bluetooth
USE_BLUETOOTH_BCM4343 := true
BOARD_HAVE_BLUETOOTH := true
BOARD_HAVE_BLUETOOTH_SPRD := true
BOARD_BLUETOOTH_BDROID_BUILDCFG_INCLUDE_DIR := device/samsung/gtexslte/bluetooth
BOARD_CUSTOM_BT_CONFIG := device/samsung/gtexslte/bluetooth/libbt_vndcfg.txt
SPRD_WCNBT_CHISET := marlin
BOARD_SPRD_WCNBT_MARLIN := true
BOARD_HAVE_FM_TROUT := true
BOARD_USE_SPRD_FMAPP := true
SPRD_CP_LOG_WCN := MARLIN
WCN_EXTENSION := true

BOARD_WPA_SUPPLICANT_DRIVER := NL80211
WPA_SUPPLICANT_VERSION      := VER_2_1_DEVEL
BOARD_WPA_SUPPLICANT_PRIVATE_LIB := lib_driver_cmd_sprdwl
BOARD_HOSTAPD_DRIVER        := NL80211
BOARD_HOSTAPD_PRIVATE_LIB   := lib_driver_cmd_sprdwl
BOARD_WLAN_DEVICE           := sc2341
WIFI_DRIVER_FW_PATH_PARAM   := "/data/misc/wifi/fwpath"
WIFI_DRIVER_FW_PATH_STA     := "sta_mode"
WIFI_DRIVER_FW_PATH_P2P     := "p2p_mode"
WIFI_DRIVER_FW_PATH_AP      := "ap_mode"
WIFI_DRIVER_MODULE_PATH     := "/lib/modules/sprdwl.ko"
WIFI_DRIVER_MODULE_NAME     := "sprdwl"

# TARGET_PREBUILT_KERNEL := kernel/samsung/gtexslte/arch/arm/boot/zImage

# Charger
BOARD_CHARGER_ENABLE_SUSPEND := true
BOARD_CHARGING_MODE_BOOTING_LPM := /sys/class/power_supply/battery/batt_lp_charging

# Integrated kernel building configs

TARGET_KERNEL_SOURCE := kernel/samsung/gtexslte
TARGET_KERNEL_CONFIG := gtexslte_defconfig
TARGET_VARIANT_CONFIG := gtexslte_defconfig
TARGET_SELINUX_CONFIG := gtexslte_defconfig
TARGET_KERNEL_CROSS_COMPILE_PREFIX := arm-linux-androideabi-
# BOARD_MKBOOTIMG_ARGS := --base 0 --pagesize 2048
BOARD_CUSTOM_BOOTIMG_MK := $(LOCAL_PATH)/mkbootimg.mk

# #
SC9830_MODULES:
	mkdir -p $(PRODUCT_OUT)/root/lib/modules
	mkdir -p $(PRODUCT_OUT)/recovery/root/lib/modules
	make -C $(TARGET_KERNEL_SOURCE)/external_module/mali MALI_PLATFORM=sc8830 BUILD=release KDIR=$(KERNEL_OUT)
	cp $(TARGET_KERNEL_SOURCE)/external_module/mali/*.ko $(PRODUCT_OUT)/root/lib/modules
	cp $(TARGET_KERNEL_SOURCE)/external_module/mali/*.ko $(PRODUCT_OUT)/recovery/root/lib/modules
	make -C $(TARGET_KERNEL_SOURCE)/external_module/wifi KDIR=$(KERNEL_OUT) clean
	make -C $(TARGET_KERNEL_SOURCE)/external_module/wifi KDIR=$(KERNEL_OUT)
	cp $(TARGET_KERNEL_SOURCE)/external_module/wifi/*.ko $(PRODUCT_OUT)/root/lib/modules
	cp $(TARGET_KERNEL_SOURCE)/external_module/wifi/*.ko $(PRODUCT_OUT)/recovery/root/lib/modules
	find -L ${KERNEL_OUT}/drivers -name "*.ko" -exec cp -f {} $(PRODUCT_OUT)/root/lib/modules \;

TARGET_KERNEL_MODULES := SC9830_MODULES

# Enable WEBGL in WebKit
ENABLE_WEBGL := true

BOARD_SEPOLICY_DIRS += device/samsung/gtexslte/sepolicy

# Camera
#zsl capture
TARGET_BOARD_CAMERA_CAPTURE_MODE := false

#back camera rotation capture
TARGET_BOARD_BACK_CAMERA_ROTATION := false

#front camera rotation capture
TARGET_BOARD_FRONT_CAMERA_ROTATION := false

#rotation capture
TARGET_BOARD_CAMERA_ROTATION_CAPTURE := false

# camera sensor type
CAMERA_SENSOR_TYPE_BACK := "s5k4ecgx_mipi"
CAMERA_SENSOR_TYPE_FRONT := "sr352_mipi"

# select camera 2M,3M,5M,8M
CAMERA_SUPPORT_SIZE := 5M
FRONT_CAMERA_SUPPORT_SIZE := 2M

TARGET_BOARD_NO_FRONT_SENSOR := true
TARGET_BOARD_CAMERA_FLASH_CTRL := false

#read otp method 1:from kernel 0:from user
TARGET_BOARD_CAMERA_READOTP_METHOD := 1

#face detect
TARGET_BOARD_CAMERA_FACE_DETECT := true

#sensor interface
TARGET_BOARD_BACK_CAMERA_INTERFACE := mipi
TARGET_BOARD_FRONT_CAMERA_INTERFACE := mipi

#select camera zsl cap mode
TARGET_BOARD_CAMERA_CAPTURE_MODE := true

#select camera zsl force cap mode
TARGET_BOARD_CAMERA_FORCE_ZSL_MODE := true

#sprd zsl feature
TARGET_BOARD_CAMERA_SPRD_PRIVATE_ZSL := true

#rotation capture
TARGET_BOARD_CAMERA_ROTATION_CAPTURE := false

#select camera support autofocus
TARGET_BOARD_CAMERA_AUTOFOCUS := true

TARGET_BOARD_CAMERA_FACE_BEAUTY := false

#uv denoise enable
TARGET_BOARD_CAMERA_CAPTURE_DENOISE := false

#y denoise enable
TARGET_BOARD_CAMERA_Y_DENOISE := true

#select continuous auto focus
TARGET_BOARD_CAMERA_CAF := true

TARGET_BOARD_CAMERA_NO_FLASH_DEV := false

#image angle in different project
TARGET_BOARD_CAMERA_ADAPTER_IMAGE := 0

#pre_allocate capture memory
TARGET_BOARD_CAMERA_PRE_ALLOC_CAPTURE_MEM := true

#sc8830g isp ver 0;sc9630 isp ver 1;sp9832a_2h11 isp version 2
TARGET_BOARD_CAMERA_ISP_SOFTWARE_VERSION := 2

TARGET_BOARD_CAMERA_ISP_AE_VERSION := 0

#set hal version to 1.0
TARGET_BOARD_CAMERA_HAL_VERSION := 1.0

#support auto anti-flicker
TARGET_BOARD_CAMERA_ANTI_FLICKER := true

#multi cap memory mode
TARGET_BOARD_MULTI_CAP_MEM := true

#low capture memory
TARGET_BOARD_LOW_CAPTURE_MEM := true

#select mipi d-phy mode(none, phya, phyb, phyab)
TARGET_BOARD_FRONT_CAMERA_MIPI := phyb
TARGET_BOARD_BACK_CAMERA_MIPI := phya

#select ccir pclk src(source0, source1)
TARGET_BOARD_FRONT_CAMERA_CCIR_PCLK := source0
TARGET_BOARD_BACK_CAMERA_CCIR_PCLK := source0


# misc
TARGET_HAS_BACKLIT_KEYS := false

# RECOVERY_VARIANT := twrp

TW_THEME := portrait_hdpi
TW_HAS_DOWNLOAD_MODE := true
TW_NO_REBOOT_BOOTLOADER := true
TW_EXCLUDE_SUPERSU := true
TW_BRIGHTNESS_PATH := "/sys/devices/gen-panel-backlight.29/backlight/panel/brightness"
TW_MAX_BRIGHTNESS := 255
TW_DEFAULT_BRIGHTNESS := 162
TARGET_USE_CUSTOM_LUN_FILE_PATH := "/sys/devices/20200000.usb/gadget/lun0/file"
TARGET_RECOVERY_FSTAB = device/samsung/gtexslte/recovery.fstab
TARGET_RECOVERY_PIXEL_FORMAT := "ABGR_8888"
RECOVERY_GRAPHICS_FORCE_USE_LINELENGTH := true
RECOVERY_GRAPHICS_FORCE_SINGLE_BUFFER := true
RECOVERY_SDCARD_ON_DATA := true
BOARD_HAS_NO_MISC_PARTITION := true
BOARD_HAS_FLIPPED_SCREEN := true
BOARD_HAS_NO_SELECT_BUTTON := true

# Encryption support
TW_INCLUDE_CRYPTO := true

# Build system
USE_NINJA := false

# Use dmalloc() for such low memory devices like us
MALLOC_SVELTE := true
BOARD_USES_LEGACY_MMAP := true

# Bionic
TARGET_NEEDS_PLATFORM_TEXT_RELOCATIONS := true

# SELinux
SERVICES_WITHOUT_SELINUX_DOMAIN := true
