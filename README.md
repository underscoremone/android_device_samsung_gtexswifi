Device Tree for the Samsung Galaxy TAB A (2016) SM-T285

Hardware Information
====================

|-----------------|-----------------------------|
| Device Name     | Samsung Galaxy Tab A6 2016 7" LTE|
|-----------------|-----------------------------|
|Model            | SM-T285                     |
|-----------------|-----------------------------|
|Codename         | gtexslte                    |
|-----------------|-----------------------------|
|Chipset          |Spreadtrum sc9830 (SCX35L)   |
|-----------------|-----------------------------|
|CPU              |Spreadtrum sc8830 4x Arm Cortex A7 cores @1.5Ghz max each|
|-----------------|-----------------------------|
|Camera           |5MP back/ 2MP front, no flash|
|-----------------|-----------------------------|
|Memory (RAM)     | 1.5GB                       |
|-----------------|-----------------------------|
|Storage          |8GB eMMC flash (2GB system, 5GB data,~1GB boot, recovery, others) |
|-----------------|-----------------------------|
|GPU              |Mali 400 MP2                 |
|-----------------|-----------------------------|
|Display          |1280x800 IPS LCD             |
|-----------------|-----------------------------|
|Platform         |sc8830                       |
|-----------------|-----------------------------|
|GPS              |slsi harrier                 |
|-----------------|-----------------------------|
|Bluetooth/wifi   |sc2331/marlin                |
|-----------------|-----------------------------|
|Touch            |Melfas MCS8040               |
|-----------------|-----------------------------|

LineageOS 14.1 Building Guide
------------------------------

There are some issues with the stock bluetooth app and library used by LineageOS 14.1

These are ported over from OMNI 7.0 souces which is close to AOSP.

Steps

1. Prepare build environment for LineageOS 14.1 (https://forum.xda-developers.com/chef-central/android/how-to-build-lineageos-14-1-t3551484)

Note: For initializing the repo you can use the command below instead:

 repo init -u https://github.com/jedld/android.git -b cm-14.1
 repo sync
 
If you haven't downloaded LineageOS before this can really take a while depending on your internet connection.

Start build

```
  source build/envsetup.sh
  brunch gtexslte
```

Build kernel separately (Recommended)

The kernel should automatically be built during brunch, but you can build the kernel manually if you want.

```
cd kernel/samsung/gtexslte
./build_kernel.sh
```

Prepare flashable images and use heimdall for flashing:

assuming /home/jedld/android-work is where the lineageos repo is

boot.img ->

```
mkbootfs /home/jedld/android-work/out/target/product/gtexslte/root | minigzip > boot_kitchen/boot.img-ramdisk-new.gz

degas-mkbootimg -o boot.img --base 0 --pagesize 2048 \
  --kernel /home/jedld/android-work/kernel/samsung/gtexslte/arch/arm/boot/zImage --cmdline "console=ttyS1,115200n8" \
  --ramdisk boot_kitchen/boot.img-ramdisk-new.gz --dt /home/jedld/android-work/kernel/samsung/gtexslte/dt.img \
  --signature seandroid.img
```

Note: degas-mkbootimg is a modified version of mkbootimg for some samsung devices.

system.img
```
make_ext4fs -s -l 2147483648 -S ~/android-work/out/target/product/gtexslte/root/file_contexts -a system system.img ~/android-work/out/target/product/gtexslte/system
```

Flash using heimdall
--------------------

```
heimdall flash --KERNEL boot.img
heimdall flash --SYSTEM system.img
```

Repositories
============

Take a look at gtexslte.xml under https://github.com/jedld/android.git for dependent repositories.
