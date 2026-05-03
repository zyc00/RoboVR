# Quest Testing

One-time setup:

1. Enable Developer Mode for the Quest 3 in the Meta mobile app.
2. Connect Quest 3 to Ubuntu with a USB-C data cable.
3. Accept the USB debugging prompt inside the headset.
4. Verify:

```bash
adb devices
```

Expected state:

```text
<serial>    device
```

Run the host bridge:

```bash
cmake -S . -B build
cmake --build build
./build/host_bridge_demo 7777
```

In another terminal, configure the tunnel:

```bash
adb reverse tcp:7777 tcp:7777
```

After the APK exists, install and launch:

```bash
adb install -r quest_client/app/build/outputs/apk/debug/app-debug.apk
adb shell am start -n com.yuchen.robovr/.VrActivity
adb logcat -s RoboVR
```
