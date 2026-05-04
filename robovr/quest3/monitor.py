"""CLI monitor for Quest 3 raw input state."""

from __future__ import annotations

import argparse
import time

from .server import Quest3Server


def _fmt_pose(label: str, pose) -> str:
    if pose is None:
        return f"{label}=None"
    p = pose.position
    q = pose.quat_xyzw
    return (
        f"{label}_pos=({p[0]:+.3f},{p[1]:+.3f},{p[2]:+.3f}) "
        f"{label}_quat=({q[0]:+.3f},{q[1]:+.3f},{q[2]:+.3f},{q[3]:+.3f})"
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=7777)
    parser.add_argument("--adb-reverse", action="store_true")
    parser.add_argument("--adb-path", default="adb")
    parser.add_argument("--hz", type=float, default=10.0)
    args = parser.parse_args()

    server = Quest3Server(
        host=args.host,
        port=args.port,
        adb_reverse=args.adb_reverse,
        adb_path=args.adb_path,
    )
    server.start()
    period = 1.0 / args.hz if args.hz > 0 else 0.1
    try:
        while True:
            state = server.latest()
            if state is None:
                print("connected=False state=None")
            else:
                print(
                    f"connected={state.connected} "
                    f"{_fmt_pose('head', state.head)} "
                    f"{_fmt_pose('right_grip', state.right_grip)} "
                    f"right_trigger={state.right_trigger:.3f} "
                    f"right_squeeze={state.right_squeeze:.3f} "
                    f"A/B/X/Y={int(state.button_a)}/{int(state.button_b)}/{int(state.button_x)}/{int(state.button_y)}"
                )
            time.sleep(period)
    except KeyboardInterrupt:
        return 0
    finally:
        server.close()


if __name__ == "__main__":
    raise SystemExit(main())
