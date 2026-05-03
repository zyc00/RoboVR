# RoboVR

Quest 3 teleoperation prototype for Ubuntu hosts.

Current milestone: bring up the wired development loop.

- `host_bridge/`: Ubuntu C++ TCP bridge demo.
- `proto/`: shared packet header and packet type definitions.
- `quest_client/`: Android/Quest client skeleton.
- `tools/`: ADB and local launch helpers.
- `docs/`: design notes and test procedures.

## Host smoke test

Install the host compiler toolchain if needed:

```bash
sudo apt update
sudo apt install build-essential
```

```bash
cmake -S . -B build
cmake --build build
./build/host_bridge_demo 7777
```

The first Quest client build sends `HELLO` and `HEARTBEAT` packets over `adb reverse tcp:7777 tcp:7777`.

## Quest prerequisites

```bash
adb devices
adb reverse tcp:7777 tcp:7777
```

`adb devices` must show the Quest as `device`, not `unauthorized` or `no permissions`.

## Quest client build

This repo currently contains Gradle project files but not a checked-in Gradle wrapper. Build with Android Studio, or install Gradle/Android SDK and run:

```bash
cd quest_client
gradle assembleDebug
```

The initial APK is a transport bring-up client. It opens a regular Android Activity on Quest and sends `HELLO`/`HEARTBEAT` packets to the Ubuntu host. Native OpenXR rendering is the next milestone after this loop is verified.

## ToolHang Quest teleop

Use the fixed launcher instead of hand-building the long command:

```bash
tools/run_tool_hang_quest.sh
```

The launcher configures `adb reverse tcp:7777 tcp:7777` and starts ToolHang with H.264 stereo video, head tracking, and right-controller arm teleop.

Controls:

- `A` or `X`: reset the headset neutral frame. Head camera motion and arm motion share this frame.
- Right squeeze: hold to clutch and move the arm.
- Right trigger: press once to toggle gripper open/closed.
- `B` or `Y`: reset the arm clutch neutral while keeping the shared headset frame.
