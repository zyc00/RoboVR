# RoboVR

Quest 3 teleoperation prototype for Ubuntu hosts.

Current milestone: bring up the wired development loop.

- `host_bridge/`: Ubuntu C++ TCP bridge demo.
- `proto/`: shared packet header and packet type definitions.
- `quest_client/`: Android/Quest client skeleton.
- `examples/quest_streaming.py`: shared Python protocol and H.264 streaming helpers.
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

## Quest 3 Raw Python API

RoboInfra should consume the raw Quest input API from `robovr.quest3`. This API only exposes headset/controller state and connection stats; it does not own robot teleop mapping.

```python
from robovr.quest3 import Quest3Server, Quest3VideoStreamer

server = Quest3Server(port=7777, adb_reverse=True)
server.start()
streamer = Quest3VideoStreamer(server)

state = server.wait_for_state(timeout_s=5.0)
while True:
    state = server.latest()
    if state is not None and state.connected:
        head = state.head
        right_grip = state.right_grip
        trigger = state.right_trigger
        squeeze = state.right_squeeze

    # left_rgb and right_rgb are uint8 RGB arrays with shape H x W x 3.
    streamer.send(left_rgb, right_rgb)
```

`Quest3VideoStreamer` defaults to the validated Quest 3 stereo path: 800x450
per eye, H.264, 72 FPS target, and RoboVR's internal flip/SurfaceTexture packet
handling. If the Quest is not connected, `send()` returns `False` instead of
raising.

Manual monitor:

```bash
python -m robovr.quest3.monitor --port 7777 --adb-reverse
```

## Python Package Publishing

The PyPI package is scoped to the `robovr` Python package only. Quest APK/native
client code and local build artifacts stay in the repository but are excluded
from the wheel.

See [docs/publishing.md](docs/publishing.md) for build, `twine check`, TestPyPI,
and PyPI upload commands.
