## TWRP device tree for Galaxy Tab A 7.0 - SM-T280 (gtexswifi)

Add to `.repo/local_manifests/gtexswifi.xml`:

```xml
<?xml version="1.0" encoding="UTF-8"?>
<manifest>
	<project path="device/samsung/gtexswifi" name="underscoremone/android_device_samsung_gtexswifi" remote="github" revision="android-6.0" />
</manifest>
```

Then run `repo sync` to check it out.

To build:

```sh
. build/envsetup.sh
lunch omni_gtexswifi-eng
make -j5 recoveryimage
```

Kernel sources are available at: https://github.com/underscoremone/android_kernel_samsung_gtexswifi/tree/cm-13.0

