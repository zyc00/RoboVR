"""Quest 3 raw input state dataclasses and parsing."""

from __future__ import annotations

from dataclasses import dataclass, field, replace

import numpy as np


@dataclass(frozen=True)
class Pose3D:
    position: np.ndarray
    quat_xyzw: np.ndarray
    valid: bool = True
    timestamp_ns: int | None = None

    def __post_init__(self) -> None:
        position = np.asarray(self.position, dtype=np.float64)
        quat_xyzw = np.asarray(self.quat_xyzw, dtype=np.float64)
        if position.shape != (3,):
            raise ValueError(f"position must have shape (3,), got {position.shape}")
        if quat_xyzw.shape != (4,):
            raise ValueError(f"quat_xyzw must have shape (4,), got {quat_xyzw.shape}")
        object.__setattr__(self, "position", position)
        object.__setattr__(self, "quat_xyzw", quat_xyzw)


@dataclass(frozen=True)
class Quest3InputState:
    timestamp_ns: int
    connected: bool
    head: Pose3D | None
    right_grip: Pose3D | None
    right_grip_flags: int
    left_trigger: float
    right_trigger: float
    left_squeeze: float
    right_squeeze: float
    button_a: bool
    button_b: bool
    button_x: bool
    button_y: bool
    ipd_m: float | None
    eye_view_count: int | None
    last_heartbeat_ns: int | None
    last_video_ack_ns: int | None
    raw: dict[str, str] = field(default_factory=dict)


def empty_input_state(*, timestamp_ns: int, connected: bool = False) -> Quest3InputState:
    return Quest3InputState(
        timestamp_ns=timestamp_ns,
        connected=connected,
        head=None,
        right_grip=None,
        right_grip_flags=0,
        left_trigger=0.0,
        right_trigger=0.0,
        left_squeeze=0.0,
        right_squeeze=0.0,
        button_a=False,
        button_b=False,
        button_x=False,
        button_y=False,
        ipd_m=None,
        eye_view_count=None,
        last_heartbeat_ns=None,
        last_video_ack_ns=None,
        raw={},
    )


def parse_bool(value: str | None, default: bool = False) -> bool:
    if value is None:
        return default
    return value.strip().lower() in {"1", "true", "yes", "y", "on"}


def parse_float(value: str | None, default: float = 0.0) -> float:
    if value is None:
        return default
    try:
        return float(value)
    except ValueError:
        return default


def parse_int(value: str | None, default: int = 0) -> int:
    if value is None:
        return default
    try:
        return int(value, 0)
    except ValueError:
        return default


def parse_optional_float(value: str | None) -> float | None:
    if value is None:
        return None
    try:
        return float(value)
    except ValueError:
        return None


def parse_optional_int(value: str | None) -> int | None:
    if value is None:
        return None
    try:
        return int(value, 0)
    except ValueError:
        return None


def pose_from_raw(
    raw: dict[str, str],
    *,
    prefix: str,
    timestamp_ns: int | None = None,
    valid: bool = True,
) -> Pose3D | None:
    keys = [
        f"{prefix}_px",
        f"{prefix}_py",
        f"{prefix}_pz",
        f"{prefix}_qx",
        f"{prefix}_qy",
        f"{prefix}_qz",
        f"{prefix}_qw",
    ]
    if any(key not in raw for key in keys):
        return None
    try:
        position = np.asarray(
            [float(raw[f"{prefix}_px"]), float(raw[f"{prefix}_py"]), float(raw[f"{prefix}_pz"])],
            dtype=np.float64,
        )
        quat = np.asarray(
            [float(raw[f"{prefix}_qx"]), float(raw[f"{prefix}_qy"]), float(raw[f"{prefix}_qz"]), float(raw[f"{prefix}_qw"])],
            dtype=np.float64,
        )
    except ValueError:
        return None
    return Pose3D(position=position, quat_xyzw=quat, valid=valid, timestamp_ns=timestamp_ns)


def update_state_from_pose_payload(
    previous: Quest3InputState | None,
    raw: dict[str, str],
    *,
    timestamp_ns: int,
    connected: bool = True,
) -> Quest3InputState:
    base = previous or empty_input_state(timestamp_ns=timestamp_ns, connected=connected)
    return replace(
        base,
        timestamp_ns=timestamp_ns,
        connected=connected,
        head=pose_from_raw(raw, prefix="head", timestamp_ns=timestamp_ns),
        right_grip=pose_from_raw(raw, prefix="right_grip", timestamp_ns=timestamp_ns),
        right_grip_flags=parse_int(raw.get("right_grip_flags"), base.right_grip_flags),
        left_trigger=parse_float(raw.get("left_trigger"), base.left_trigger),
        right_trigger=parse_float(raw.get("right_trigger"), base.right_trigger),
        left_squeeze=parse_float(raw.get("left_squeeze"), base.left_squeeze),
        right_squeeze=parse_float(raw.get("right_squeeze"), base.right_squeeze),
        button_a=parse_bool(raw.get("button_a"), base.button_a),
        button_b=parse_bool(raw.get("button_b"), base.button_b),
        button_x=parse_bool(raw.get("button_x"), base.button_x),
        button_y=parse_bool(raw.get("button_y"), base.button_y),
        ipd_m=parse_optional_float(raw.get("ipd_m")),
        eye_view_count=parse_optional_int(raw.get("eye_view_count")),
        raw=dict(raw),
    )

