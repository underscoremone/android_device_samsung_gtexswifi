Device Tree for the Samsung Galaxy TAB A (2016) SM-T285

Hardware Information
====================

Device Name:      Samsung Galaxy Tab A6 7" LTE
Model:            SM-T285
Codename:         gtexslte
Chipset:          Spreadtrum sc9830
CPU:              Spreadtrum sc8830 4 cores @1.5Ghz max each
Memory (RAM):     1.5GB
Storage:          8GB eMMC flash (2GB system, 5GB data,~1GB boot, recovery, others)
GPU:              Mali 400 MP2
Display:          1280x800 IPS LCD
Platform:         sc8830
GPS:              slsi harrier
Bluetooth/wifi:   sc2331/marlin
Touch:            Melfas MCS8040

LineageOS 14.1 Building Guide
------------------------------

Download dependent repos

Spreadtrum hardware libraries and drivers
-----------------------------

https://github.com/jedld/hardware_sprd.git


There are some issues with the stock bluetooth app and library used by LineageOS 14.1

download the custom bluetooth packages from the following repos:

https://github.com/jedld/packages_apps_Bluetooth
https://github.com/jedld/system_bt

These are ported over from OMNI 7.0 souces which is close to AOSP.

Prepare Build environment, using an Ubuntu 16.10 system

- Prepare build environment for LineageOS 14.1
- Download this device tree into devices/samsung/gtexslte
- Download kernel sources into kernel/samsung/gtexslte
- Download vendor proprietary block into vendor/samsung/gtexste
- Download spreadtrum open source into /vendor/sprd
- apply source code patches from devices/samsung/gtexslte/patches


Start build

```
  source build/envsetup.sh
  brunch gtexslte
```

Build kernel separately (Recommended)

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

Kernel
------
```
https://github.com/jedld/kernel_samsung_gtexslte.git -b performance
```

Vendor Tree
-----------
```
https://github.com/jedld/vendor_samsung_gtexslte.git
```

Spreadtrum Opensource
-----------------
```
https://github.com/jedld/android_vendor_sprd.git -b lineageos-14.1
```
