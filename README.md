### LineageOS device tree for Samsung Galaxy Tab A 7.0 (2016) (gtexswifi)

Component   | Details
-------:|:-------------------------
Model   | SM-T280
CPU     | Quad-core 1.30GHz Spreadtrum SC7730SW
GPU     | Mali-400MP2
Memory  | 1.5GB RAM
Shipped Android Version | 5.1.1
Released | March 2016
Storage | 8GB
MicroSD Card Slot | Present, up to 256GB
Battery | 4000 mAh
Display | 7.0" 800 x 1280 px (216 dpi)
Primary Camera  | 5 MP, f/2.2, autofocus
Secondary Camera | 2.0 MP, f/2.2
FM Radio | Present

<img height="600" src="https://images.samsung.com/is/image/samsung/nz-galaxy-tab-a-7-0-2016-t280-sm-t280nzkaxnz-000000001-front-black?$L2-Thumbnail$" title="Galaxy Tab A 7.0"/>

---

### Build Instructions
If you want to compile LineageOS-14.1 for gtexswifi follow [LinegeOS Getting started](https://github.com/LineageOS/android/tree/cm-14.1#getting-started) instructions and use this roomservice.xml:
```xml
<?xml version="1.0" encoding="UTF-8"?>
<manifest>
    <remove-project name="LineageOS/android_system_bt" />
    <remove-project name="LineageOS/android_packages_apps_Bluetooth" />
    <project path="system/bt" name="lineage-gtexslte/system_bt" remote="github" revision="aosp-7.1"/>
    <project path="packages/apps/Bluetooth" name="jedld/packages_apps_Bluetooth" remote="github" revision="master"/>
    <project path="external/stlport" name="LineageOS/android_external_stlport" remote="github" revision="cm-14.1"/>
    <project path="hardware/samsung" name="LineageOS/android_hardware_samsung" remote="github" revision="cm-14.1"/>
    <project path="kernel/samsung/gtexswifi" name="underscoremone/android_kernel_samsung_gtexswifi" remote="github" revision="cm-14.1"/>
    <project path="device/samsung/gtexswifi" name="underscoremone/android_device_samsung_gtexswifi" remote="github" revision="cm-14.1"/>
    <project path="vendor/samsung/gtexswifi" name="underscoremone/proprietary_vendor_samsung_gtexswifi" remote="github" revision="cm-14.1"/>
    <project path="hardware/sprd" name="underscoremone/android_hardware_sprd" remote="github" revision="cm-14.1"/>
    <project path="packages/apps/SamsungServiceMode" name="LineageOS/android_packages_apps_SamsungServiceMode" remote="github" revision="cm-14.1"/>
    <project path="prebuilts/gcc/linux-x86/arm/arm-eabi-7.5" name="underscoremone/android_prebuilts_gcc_linux-x86_arm_arm-eabi" remote="github" revision="linaro-7.5.0-2019.12" />
</manifest>
```

If you want the keyboard with the 5th row with numbers add this to roomservice.xml:
```xml
    <remove-project name="LineageOS/android_packages_inputmethods_LatinIME" />
    <project path="packages/inputmethods/LatinIME" name="underscoremone/android_packages_inputmethods_LatinIME" remote="github" revision="_cm-14.1"/>
```

To build:
```sh
. build/envsetup.sh
breakfast lineage_gtexswifi-userdebug && make bacon
```

---

If you want to compile TWRP for gtexswifi add this to roomservice.xml:
```xml
    <project path="bootable/recovery-twrp" name="TeamWin/android_bootable_recovery" remote="github" revision="android-9.0" />
    <project path="external/busybox" name="LineageOS/android_external_busybox" remote="github" revision="cm-14.1" />
```

To build:
```sh
export WITH_TWRP=true
```
```sh
. build/envsetup.sh
breakfast lineage_gtexswifi-eng && make recoveryimage
```

TWRP can't be compiled together with the ROM because for the boot.img we have to use uncompressed kernel (Image), and if we use it for the recovery.img the build will fail because the img will result too big, if we compress the recovery ramdisk with lzma the build will be successfull but the recovery won't boot. So just compile it separately (using compressed kernel (zImage)) with the command above, and ignore the recovery.img (lineage-recovery) generated while compiling the ROM.

---
