### BoardConfig.mk ###

LOCAL_PATH := device/samsung/gtexswifi

TARGET_OTA_ASSERT_DEVICE := SM-T280,gtexswifi

# SPRD hardware
BOARD_USES_SPRD_HARDWARE := true
TARGET_SPRD_HARDWARE := true
SOC_SCX30G_V2 := true

# Platform
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

# Partitions
BOARD_BOOTIMAGE_PARTITION_SIZE     := 16777216
BOARD_RECOVERYIMAGE_PARTITION_SIZE := 16777216
BOARD_SYSTEMIMAGE_PARTITION_SIZE   := 2147483648
BOARD_USERDATAIMAGE_PARTITION_SIZE := 5289017344
BOARD_CACHEIMAGE_PARTITION_SIZE    := 209715200
BOARD_FLASH_BLOCK_SIZE := 131072
TARGET_USERIMAGES_USE_EXT4 := true
#TARGET_USERIMAGES_USE_F2FS := true
BOARD_HAS_LARGE_FILESYSTEM := true
BOARD_CACHEIMAGE_FILE_SYSTEM_TYPE := ext4

# Kernel
BOARD_CUSTOM_BOOTIMG := true
BOARD_CUSTOM_BOOTIMG_MK := device/samsung/gtexswifi/mkbootimg.mk
BOARD_KERNEL_CMDLINE := console=ttyS1,115200n8 androidboot.selinux=permissive
BOARD_KERNEL_BASE := 0x00000000
BOARD_KERNEL_PAGESIZE := 2048
BOARD_MKBOOTIMG_ARGS := --base 0 --pagesize 2048 --kernel_offset 0x00008000 --ramdisk_offset 0x01000000 --tags_offset 0x00000100
BOARD_KERNEL_SEPARATED_DT := true
TARGET_KERNEL_CROSS_COMPILE_PREFIX := arm-eabi-
KERNEL_TOOLCHAIN := $(ANDROID_BUILD_TOP)/prebuilts/gcc/linux-x86/arm/arm-eabi-7.5/bin
TARGET_KERNEL_SOURCE := kernel/samsung/gtexswifi
ifeq ($(WITH_TWRP),true)
-include device/samsung/gtexswifi/twrp.mk
else
BOARD_KERNEL_IMAGE_NAME := Image
TARGET_KERNEL_CONFIG := gtexswifi-dt_defconfig
TARGET_KERNEL_SELINUX_CONFIG := gtexswifi-dt_defconfig
TARGET_VARIANT_CONFIG := gtexswifi-dt_defconfig
NEED_KERNEL_MODULE_ROOT := true

TARGET_KERNEL_MODULES := SPRDWL_MODULE

SPRDWL_MODULE:
	mv $(KERNEL_OUT)/drivers/net/wireless/sc2331/sprdwl.ko $(KERNEL_MODULES_OUT)

TARGET_RECOVERY_FSTAB = device/samsung/gtexswifi/rootdir/recovery.fstab
LZMA_RAMDISK_TARGETS := recovery
endif

# sdFAT filesystem for exFAT
TARGET_KERNEL_HAVE_EXFAT := true

# Audio
SPRD_AUDIO_USE_NEW_API := true
BOARD_USES_TINYALSA_AUDIO := true
BOARD_USES_SS_VOIP := true
BOARD_USE_LIBATCHANNEL_WRAPPER := true

# something
BOARD_NEEDS_MEMORYHEAPION_SPRD := true

# Codecs
BOARD_CANT_REALLOCATE_OMX_BUFFERS := true

# PowerHAL
TARGET_POWERHAL_VARIANT := scx35

# RIL
BOARD_PROVIDES_RILD := false
BOARD_PROVIDES_LIBRIL := false

# Graphics
TARGET_RUNNING_WITHOUT_SYNC_FRAMEWORK := true
BOARD_EGL_NEEDS_HANDLE_VALUE := true
TARGET_REQUIRES_SYNCHRONOUS_SETSURFACE := true
TARGET_FORCE_SCREENSHOT_CPU_PATH := true
NUM_FRAMEBUFFER_SURFACE_BUFFERS := 3
TARGET_GPU_USE_TILE_ALIGN := true
USE_UI_OVERLAY := true
USE_SPRD_DITHER := true
TARGET_GPU_PLATFORM := utgard
TARGET_USE_PREBUILT_GRALLOC := false
ARCH_ARM_HAVE_TLS_REGISTER := true
BOARD_EGL_CFG := device/samsung/gtexswifi/configs/egl.cfg

#1080 hw acceleration
BOARD_VSP_SUPPORT_1080I := true

# HWComposer
USE_SPRD_HWCOMPOSER := true
USE_OVERLAY_COMPOSER_GPU := true
TARGET_FORCE_HWC_FOR_VIRTUAL_DISPLAYS := true

BOARD_GLOBAL_CFLAGS += -DDISABLE_ASHMEM_TRACKING

# Bluetooth
USE_BLUETOOTH_BCM4343 := true
BOARD_HAVE_BLUETOOTH := true
BOARD_HAVE_BLUETOOTH_SPRD := true
BOARD_BLUETOOTH_BDROID_BUILDCFG_INCLUDE_DIR := device/samsung/gtexswifi/configs/bluetooth
BOARD_CUSTOM_BT_CONFIG := device/samsung/gtexswifi/bluetooth/configs/libbt_vndcfg.txt
SPRD_WCNBT_CHISET := marlin
BOARD_SPRD_WCNBT_MARLIN := true
BOARD_HAVE_FM_TROUT := true
BOARD_USE_SPRD_FMAPP := true
SPRD_CP_LOG_WCN := MARLIN
WCN_EXTENSION := true

# WiFi
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

# Charger
BOARD_CHARGER_ENABLE_SUSPEND := true
BOARD_CHARGING_MODE_BOOTING_LPM := /sys/class/power_supply/battery/batt_lp_charging

# Enable WEBGL in WebKit
ENABLE_WEBGL := true

# SELinux
BOARD_SEPOLICY_DIRS += device/samsung/gtexswifi/sepolicy
SERVICES_WITHOUT_SELINUX_DOMAIN := true

# Camera
#zsl capture
TARGET_BOARD_CAMERA_CAPTURE_MODE := false

#back camera rotation capture
TARGET_BOARD_BACK_CAMERA_ROTATION := false

#front camera rotation capture
TARGET_BOARD_FRONT_CAMERA_ROTATION := false

#rotation capture
TARGET_BOARD_CAMERA_ROTATION_CAPTURE := false

# camera sensor type  #S5K4ECGA  #SR259
CAMERA_SENSOR_TYPE_BACK := "s5k4ecgx_mipi"
CAMERA_SENSOR_TYPE_FRONT := "sr352_mipi"

# select camera 2M,3M,5M,8M
CAMERA_SUPPORT_SIZE := 5M
FRONT_CAMERA_SUPPORT_SIZE := 2M

TARGET_BOARD_NO_FRONT_SENSOR := false
TARGET_BOARD_CAMERA_FLASH_CTRL := false

#read otp method 1:from kernel 0:from user
TARGET_BOARD_CAMERA_READOTP_METHOD := 1

#face detect
TARGET_BOARD_CAMERA_FACE_DETECT := false

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

TARGET_BOARD_CAMERA_NO_FLASH_DEV := true

#image angle in different project
TARGET_BOARD_CAMERA_ADAPTER_IMAGE := 0

#pre_allocate capture memory
TARGET_BOARD_CAMERA_PRE_ALLOC_CAPTURE_MEM := true

#sc8830g isp ver 0;sc9630 isp ver 1;sp9832a_2h11 isp version 2
TARGET_BOARD_CAMERA_ISP_SOFTWARE_VERSION := 2

TARGET_BOARD_CAMERA_ISP_AE_VERSION := 0

#set hal version to 1.0
TARGET_USES_MEDIA_EXTENSIONS := true
TARGET_BOARD_CAMERA_HAL_VERSION := 1.0

TARGET_BOARD_USE_THRID_LIB := true
TARGET_BOARD_USE_THIRD_AWB_LIB_A := true
TARGET_BOARD_USE_ALC_AE_AWB := false
TARGET_BOARD_USE_THIRD_AF_LIB_A := true

TARGET_VCM_BU64241GWZ := true

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

# System properties
TARGET_SYSTEM_PROP += device/samsung/gtexswifi/system.prop

# Use dmalloc() for such low memory devices like us
MALLOC_SVELTE := true
BOARD_USES_LEGACY_MMAP := true

# Bionic
TARGET_NEEDS_PLATFORM_TEXT_RELOCATIONS := true

