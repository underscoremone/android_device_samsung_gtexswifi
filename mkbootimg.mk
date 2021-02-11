LOCAL_PATH := $(call my-dir)

## Build and run dtbtool
DTBTOOL := $(HOST_OUT_EXECUTABLES)/dtbToolCM$(HOST_EXECUTABLE_SUFFIX)
INSTALLED_DTIMAGE_TARGET := $(PRODUCT_OUT)/dt.img
DTBTAGNAME := "sprd,sc-id = <"

INSHEAD := $(LOCAL_PATH)/insertheader/prebuilt/bin/imgheaderinsert
SIGNEDBIMG ?= $(PRODUCT_OUT)/boot-sign.img
SIGNEDRIMG ?= $(PRODUCT_OUT)/recovery-sign.img
SPRDSIGN := $(LOCAL_PATH)/signimage/prebuilt/bin/sprd_sign
SPRDSIGNCFG := $(LOCAL_PATH)/signimage/config

ifneq ($(TARGET_KERNEL_ARCH),)
KERNEL_ARCH := $(TARGET_KERNEL_ARCH)
else
KERNEL_ARCH := $(TARGET_ARCH)
endif

$(INSTALLED_DTIMAGE_TARGET): $(DTBTOOL) $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr $(INSTALLED_KERNEL_TARGET)
	$(call pretty,"Target dt image: $(INSTALLED_DTIMAGE_TARGET)")
	$(hide) $(DTBTOOL) -o $(INSTALLED_DTIMAGE_TARGET) -s $(BOARD_KERNEL_PAGESIZE) -p $(KERNEL_OUT)/scripts/dtc/ $(KERNEL_OUT)/arch/$(KERNEL_ARCH)/boot/dts/
	@echo -e ${CL_CYN}"Made DT image: $@"${CL_RST}

# Overload bootimg generation:
$(INSTALLED_BOOTIMAGE_TARGET): $(MKBOOTIMG) $(INTERNAL_BOOTIMAGE_FILES) $(INSTALLED_DTIMAGE_TARGET)
	$(call pretty,"Target boot image: $@")
	$(hide) $(MKBOOTFS) -d $(TARGET_OUT) $(TARGET_ROOT_OUT) | $(BOOT_RAMDISK_COMPRESSOR) > $(INSTALLED_RAMDISK_TARGET)
	$(hide) $(MKBOOTIMG) $(INTERNAL_BOOTIMAGE_ARGS) $(BOARD_MKBOOTIMG_ARGS) --dt $(INSTALLED_DTIMAGE_TARGET) --output $@
	$(hide) $(call assert-max-image-size,$@,$(BOARD_BOOTIMAGE_PARTITION_SIZE),raw)
	@echo "SEANDROIDENFORCE" >> $@
	$(hide) $(INSHEAD) $(INSTALLED_BOOTIMAGE_TARGET) 1
	$(hide) mv $(SIGNEDBIMG) $(INSTALLED_BOOTIMAGE_TARGET)
	$(hide) $(SPRDSIGN) $(INSTALLED_BOOTIMAGE_TARGET) $(SPRDSIGNCFG)
	@echo -e ${CL_CYN}"Made boot image: $@"${CL_RST}

## Overload recoveryimg generation:
$(INSTALLED_RECOVERYIMAGE_TARGET): $(MKBOOTIMG) $(INSTALLED_DTIMAGE_TARGET) \
		$(recovery_ramdisk) \
		$(recovery_kernel)
	$(hide) $(call build-recoveryimage-target, $@)
	$(hide) $(MKBOOTIMG) $(INTERNAL_RECOVERYIMAGE_ARGS) $(BOARD_MKBOOTIMG_ARGS) --dt $(INSTALLED_DTIMAGE_TARGET) --output $@
	$(hide) $(call assert-max-image-size,$@,$(BOARD_RECOVERYIMAGE_PARTITION_SIZE),raw)
	$(hide) echo -n "SEANDROIDENFORCE" >> $(INSTALLED_RECOVERYIMAGE_TARGET)
	$(hide) $(INSHEAD) $(INSTALLED_RECOVERYIMAGE_TARGET) 1
	$(hide) mv $(SIGNEDRIMG) $(INSTALLED_RECOVERYIMAGE_TARGET)
	$(hide) $(SPRDSIGN) $(INSTALLED_RECOVERYIMAGE_TARGET) $(SPRDSIGNCFG)
	$(hide) tar -C $(PRODUCT_OUT) -H ustar -c recovery.img > $(PRODUCT_OUT)/recovery.tar
	@echo -e ${CL_CYN}"Made recovery image: $@"${CL_RST}
