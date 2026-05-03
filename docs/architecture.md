# Architecture

RoboVR treats Quest 3 as a native Android/OpenXR endpoint, not as a PCVR display.

```text
Ubuntu host bridge              USB-C / ADB reverse             Quest 3 APK
------------------              -------------------             -----------
TCP server on 127.0.0.1:7777  <--------------------------->      native socket client
protocol framing                                                   OpenXR renderer later
video encoder later                                                MediaCodec later
pose buffer later                                                   pose sender later
```

The first bring-up target is intentionally small:

1. Build and run `host_bridge_demo`.
2. Configure `adb reverse tcp:7777 tcp:7777`.
3. Install and launch the Quest APK.
4. Observe `HELLO` and `HEARTBEAT` packets on the host.

OpenXR rendering and controller pose sampling will be added after this transport loop is stable.
