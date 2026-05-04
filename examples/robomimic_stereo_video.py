"""Stream a robosuite / robomimic-style stereo camera to the Quest client.

This example intentionally sends raw RGBA frames through the existing RoboVR
video packet path. Keep the default resolution small enough to fit the current
1 MiB Quest packet guard. Once stereo geometry feels right, the same render
source can be connected to a real-time H.264 encoder.
"""

from __future__ import annotations

import argparse
import os
import socket
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import mujoco
from PIL import Image


ROBOCORPUS_ROOT = Path(__file__).resolve().parents[2] / "RoboCorpus"
ROBOVR_ROOT = Path(__file__).resolve().parents[1]
if str(ROBOVR_ROOT) not in sys.path:
    sys.path.insert(0, str(ROBOVR_ROOT))
if ROBOCORPUS_ROOT.exists():
    sys.path.insert(0, str(ROBOCORPUS_ROOT))

from quest_streaming import (
    H264Encoder,
    MAGIC,
    MAX_QUEST_PAYLOAD_BYTES,
    PACKET_HEADER,
    PT_HEARTBEAT,
    PT_HELLO,
    PT_POSE,
    PT_STATS,
    PT_VIDEO,
    VERSION,
    make_h264_stereo_payload,
    make_raw_stereo_payload,
    parse_kv,
    recvall,
    send_packet,
)

QUEST_LEFT_FOV = (-0.942478, 0.698132, 0.767945, -0.959931)
QUEST_RIGHT_FOV = (-0.698132, 0.942478, 0.767945, -0.959931)


@dataclass
class StereoRgbFrame:
    frame_index: int
    left: np.ndarray
    right: np.ndarray


@dataclass
class HeadPose:
    px: float
    py: float
    pz: float
    qx: float
    qy: float
    qz: float
    qw: float
    reset_requested: bool = False


@dataclass
class QuestInput:
    right_grip_flags: int
    right_grip_px: float
    right_grip_py: float
    right_grip_pz: float
    right_grip_qx: float
    right_grip_qy: float
    right_grip_qz: float
    right_grip_qw: float
    right_trigger: float
    right_squeeze: float
    button_a: bool
    button_b: bool
    button_x: bool
    button_y: bool


class LatestHeadPose:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._pose: HeadPose | None = None

    def update_from_values(self, values: dict[str, str]) -> None:
        try:
            pose = HeadPose(
                px=float(values["head_px"]),
                py=float(values["head_py"]),
                pz=float(values["head_pz"]),
                qx=float(values["head_qx"]),
                qy=float(values["head_qy"]),
                qz=float(values["head_qz"]),
                qw=float(values["head_qw"]),
                reset_requested=values.get("button_a") == "1" or values.get("button_x") == "1",
            )
        except (KeyError, ValueError):
            return
        with self._lock:
            self._pose = pose

    def get(self) -> HeadPose | None:
        with self._lock:
            return self._pose


class LatestQuestInput:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._input: QuestInput | None = None

    def update_from_values(self, values: dict[str, str]) -> None:
        try:
            sample = QuestInput(
                right_grip_flags=int(values.get("right_grip_flags", "0")),
                right_grip_px=float(values["right_grip_px"]),
                right_grip_py=float(values["right_grip_py"]),
                right_grip_pz=float(values["right_grip_pz"]),
                right_grip_qx=float(values["right_grip_qx"]),
                right_grip_qy=float(values["right_grip_qy"]),
                right_grip_qz=float(values["right_grip_qz"]),
                right_grip_qw=float(values["right_grip_qw"]),
                right_trigger=float(values.get("right_trigger", "0")),
                right_squeeze=float(values.get("right_squeeze", "0")),
                button_a=values.get("button_a") == "1",
                button_b=values.get("button_b") == "1",
                button_x=values.get("button_x") == "1",
                button_y=values.get("button_y") == "1",
            )
        except (KeyError, ValueError):
            return
        with self._lock:
            self._input = sample

    def get(self) -> QuestInput | None:
        with self._lock:
            return self._input


def reader_loop(
    conn: socket.socket,
    stop: threading.Event,
    latest_head_pose: LatestHeadPose | None,
    latest_quest_input: LatestQuestInput | None,
) -> None:
    pose_count = 0
    heartbeat_count = 0
    video_ack_count = 0
    while not stop.is_set():
        header_bytes = recvall(conn, PACKET_HEADER.size)
        if header_bytes is None:
            stop.set()
            return
        magic, version, packet_type, seq, timestamp_ns, payload_size, crc32 = PACKET_HEADER.unpack(
            header_bytes
        )
        del timestamp_ns, crc32
        if magic != MAGIC or version != VERSION or payload_size > MAX_QUEST_PAYLOAD_BYTES:
            print(f"bad packet header magic={magic:x} version={version} size={payload_size}")
            stop.set()
            return
        payload = recvall(conn, payload_size) if payload_size else b""
        if payload is None:
            stop.set()
            return

        if packet_type == PT_HELLO:
            print(f"rx HELLO seq={seq} payload={payload.decode(errors='replace')!r}")
        elif packet_type == PT_HEARTBEAT:
            heartbeat_count += 1
            if heartbeat_count == 1 or heartbeat_count % 20 == 0:
                print(f"rx heartbeat seq={seq} count={heartbeat_count}")
        elif packet_type == PT_POSE:
            pose_count += 1
            values = parse_kv(payload)
            if latest_head_pose is not None:
                latest_head_pose.update_from_values(values)
            if latest_quest_input is not None:
                latest_quest_input.update_from_values(values)
            if pose_count == 1 or pose_count % 60 == 0:
                print(
                    "rx pose"
                    f" seq={seq} count={pose_count}"
                    f" head=({values.get('head_px', '?')},"
                    f"{values.get('head_py', '?')},"
                    f"{values.get('head_pz', '?')})"
                    f" right_grip=({values.get('right_grip_px', '?')},"
                    f"{values.get('right_grip_py', '?')},"
                    f"{values.get('right_grip_pz', '?')})"
                    f" squeeze={values.get('right_squeeze', '?')}"
                    f" trigger={values.get('right_trigger', '?')}"
                )
        elif packet_type == PT_STATS:
            values = parse_kv(payload)
            if values.get("kind") == "video_ack":
                video_ack_count += 1
                if video_ack_count == 1 or video_ack_count % 60 == 0:
                    print(
                        "rx video_ack"
                        f" frame={values.get('frame_index', '?')}"
                        f" count={video_ack_count}"
                        f" quest_ms={values.get('quest_receive_to_publish_ms', '?')}"
                        f" surface={values.get('surface', '?')}"
                    )
            else:
                print(f"rx stats seq={seq} payload={payload.decode(errors='replace')!r}")


def rgb_to_rgba_bytes(rgb: np.ndarray) -> bytes:
    rgb = np.asarray(rgb, dtype=np.uint8)
    if rgb.ndim != 3 or rgb.shape[2] != 3:
        raise ValueError(f"expected HxWx3 uint8 RGB frame, got {rgb.shape}")
    alpha = np.full((*rgb.shape[:2], 1), 255, dtype=np.uint8)
    rgba = np.concatenate((rgb, alpha), axis=2)
    return np.ascontiguousarray(rgba).tobytes()


def cover_fovy_from_eye_fovs(*eye_fovs: tuple[float, float, float, float]) -> float:
    max_vertical_tan = 0.0
    for _, _, angle_up, angle_down in eye_fovs:
        max_vertical_tan = max(max_vertical_tan, abs(np.tan(angle_up)), abs(np.tan(angle_down)))
    return float(np.degrees(2.0 * np.arctan(max_vertical_tan)))


def quat_normalized_xyzw(pose: HeadPose) -> np.ndarray:
    quat = np.asarray([pose.qx, pose.qy, pose.qz, pose.qw], dtype=float)
    norm = np.linalg.norm(quat)
    if norm <= 1.0e-9:
        return np.asarray([0.0, 0.0, 0.0, 1.0], dtype=float)
    return quat / norm


def quat_conjugate_xyzw(quat: np.ndarray) -> np.ndarray:
    return np.asarray([-quat[0], -quat[1], -quat[2], quat[3]], dtype=float)


def quat_multiply_xyzw(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.asarray(
        [
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        ],
        dtype=float,
    )


def quat_rotate_xyzw(quat: np.ndarray, vector: np.ndarray) -> np.ndarray:
    q_vector = np.asarray([vector[0], vector[1], vector[2], 0.0], dtype=float)
    return quat_multiply_xyzw(
        quat_multiply_xyzw(quat, q_vector), quat_conjugate_xyzw(quat)
    )[:3]


def normalize_quat_xyzw(quat: np.ndarray) -> np.ndarray:
    norm = np.linalg.norm(quat)
    if norm <= 1.0e-9:
        return np.asarray([0.0, 0.0, 0.0, 1.0], dtype=float)
    return quat / norm


def quat_to_rotvec_xyzw(quat: np.ndarray) -> np.ndarray:
    quat = normalize_quat_xyzw(quat)
    if quat[3] < 0.0:
        quat = -quat
    vector_norm = np.linalg.norm(quat[:3])
    if vector_norm <= 1.0e-9:
        return np.zeros(3, dtype=float)
    angle = 2.0 * np.arctan2(vector_norm, quat[3])
    return quat[:3] / vector_norm * angle


def rotate_axis_angle(vector: np.ndarray, axis: np.ndarray, angle: float) -> np.ndarray:
    axis_norm = np.linalg.norm(axis)
    if axis_norm <= 1.0e-9:
        return vector
    axis = axis / axis_norm
    return (
        vector * np.cos(angle)
        + np.cross(axis, vector) * np.sin(angle)
        + axis * np.dot(axis, vector) * (1.0 - np.cos(angle))
    )


class QuestNeutralFrame:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._right = np.asarray([1.0, 0.0, 0.0], dtype=float)
        self._up = np.asarray([0.0, 1.0, 0.0], dtype=float)
        self._forward = np.asarray([0.0, 0.0, -1.0], dtype=float)

    def set_axes(self, right: np.ndarray, up: np.ndarray, forward: np.ndarray) -> None:
        with self._lock:
            self._right = right.copy()
            self._up = up.copy()
            self._forward = forward.copy()

    def axes(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        with self._lock:
            return self._right.copy(), self._up.copy(), self._forward.copy()


class QuestArmTeleop:
    POSITION_VALID_BIT = 0x2

    def __init__(self, args: argparse.Namespace, latest_input: LatestQuestInput, neutral_frame: QuestNeutralFrame):
        self.args = args
        self.latest_input = latest_input
        self.neutral_frame = neutral_frame
        self.prev_grip_position: np.ndarray | None = None
        self.prev_grip_quat: np.ndarray | None = None
        self.was_clutched = False
        self.trigger_was_pressed = False
        self.gripper_closed = False
        self.last_action = np.zeros(7, dtype=np.float32)

    def make_action(self, action_dim: int) -> np.ndarray:
        out = np.zeros(action_dim, dtype=np.float32)
        if action_dim < 7:
            return out
        sample = self.latest_input.get()
        if sample is None:
            return out

        position_valid = (sample.right_grip_flags & self.POSITION_VALID_BIT) != 0
        clutched = position_valid and sample.right_squeeze >= self.args.arm_clutch_threshold
        reset = sample.button_b or sample.button_y
        trigger_pressed = sample.right_trigger >= self.args.arm_gripper_threshold
        if trigger_pressed and not self.trigger_was_pressed:
            self.gripper_closed = not self.gripper_closed
            print("gripper toggled closed" if self.gripper_closed else "gripper toggled open")
        self.trigger_was_pressed = trigger_pressed
        grip = 1.0 if self.gripper_closed else -1.0
        if self.args.arm_invert_gripper:
            grip *= -1.0
        out[6] = grip
        grip_position = np.asarray(
            [sample.right_grip_px, sample.right_grip_py, sample.right_grip_pz],
            dtype=float,
        )
        grip_quat = normalize_quat_xyzw(
            np.asarray(
                [
                    sample.right_grip_qx,
                    sample.right_grip_qy,
                    sample.right_grip_qz,
                    sample.right_grip_qw,
                ],
                dtype=float,
            )
        )
        if not clutched:
            self.prev_grip_position = None
            self.prev_grip_quat = None
            self.was_clutched = False
            self.last_action.fill(0.0)
            self.last_action[6] = grip
            return out

        if self.prev_grip_position is None or self.prev_grip_quat is None or not self.was_clutched or reset:
            self.prev_grip_position = grip_position
            self.prev_grip_quat = grip_quat
            self.was_clutched = True
            self.last_action.fill(0.0)
            self.last_action[6] = grip
            print("arm teleop neutral reset" if reset else "arm teleop clutch engaged")
            return out

        quest_delta = grip_position - self.prev_grip_position
        quest_rot_delta = quat_multiply_xyzw(grip_quat, quat_conjugate_xyzw(self.prev_grip_quat))
        self.prev_grip_position = grip_position
        self.prev_grip_quat = grip_quat
        if np.linalg.norm(quest_delta) < self.args.arm_position_deadband_m:
            quest_delta.fill(0.0)
        quest_rotvec = quat_to_rotvec_xyzw(quest_rot_delta)
        if np.linalg.norm(quest_rotvec) < self.args.arm_rotation_deadband_rad:
            quest_rotvec.fill(0.0)
        neutral_right, neutral_up, neutral_forward = self.neutral_frame.axes()
        local_delta = np.asarray(
            [
                np.dot(quest_delta, neutral_right),
                np.dot(quest_delta, neutral_up),
                np.dot(quest_delta, neutral_forward),
            ],
            dtype=float,
        )
        local_rotvec = np.asarray(
            [
                np.dot(quest_rotvec, neutral_right),
                np.dot(quest_rotvec, neutral_up),
                np.dot(quest_rotvec, neutral_forward),
            ],
            dtype=float,
        )

        # Headset-neutral axes: +right, +up, +forward. Panda base axes in
        # robosuite: +x forward, +y left, +z up.
        dpos = np.asarray(
            [
                local_delta[2] * self.args.arm_forward_sign,
                -local_delta[0] * self.args.arm_right_sign,
                local_delta[1] * self.args.arm_up_sign,
            ],
            dtype=float,
        )
        drot = np.asarray(
            [
                local_rotvec[2] * self.args.arm_forward_sign,
                -local_rotvec[0] * self.args.arm_right_sign,
                local_rotvec[1] * self.args.arm_up_sign,
            ],
            dtype=float,
        )
        position_action = dpos * self.args.arm_position_gain / self.args.arm_osc_position_output_max
        out[:3] = np.clip(position_action, -self.args.arm_max_action, self.args.arm_max_action)
        if not self.args.arm_lock_rotation:
            rotation_action = drot * self.args.arm_rotation_gain / self.args.arm_osc_rotation_output_max
            out[3:6] = np.clip(rotation_action, -self.args.arm_max_action, self.args.arm_max_action)

        self.last_action = out[:7].copy()
        return out


def crop_to_eye_fov(
    rgb: np.ndarray,
    *,
    eye_fov: tuple[float, float, float, float],
    camera_fovy_deg: float,
    output_width: int,
    output_height: int,
) -> np.ndarray:
    """Crop a symmetric MuJoCo render into an asymmetric Quest eye frustum.

    MuJoCo's public camera path gives us a centered vertical-FOV camera. Quest
    uses per-eye asymmetric frusta. Rendering a covering symmetric frustum and
    cropping by tangent-angle ranges approximates the same projection at the
    texture we submit to the Quest swapchain.
    """

    angle_left, angle_right, angle_up, angle_down = eye_fov
    height, width = rgb.shape[:2]
    vertical_tan = float(np.tan(np.radians(camera_fovy_deg) * 0.5))
    horizontal_tan = vertical_tan * (width / height)

    x0 = (float(np.tan(angle_left)) + horizontal_tan) / (2.0 * horizontal_tan) * width
    x1 = (float(np.tan(angle_right)) + horizontal_tan) / (2.0 * horizontal_tan) * width
    y0 = (vertical_tan - float(np.tan(angle_up))) / (2.0 * vertical_tan) * height
    y1 = (vertical_tan - float(np.tan(angle_down))) / (2.0 * vertical_tan) * height

    left = int(np.clip(np.floor(x0), 0, width - 1))
    right = int(np.clip(np.ceil(x1), left + 1, width))
    top = int(np.clip(np.floor(y0), 0, height - 1))
    bottom = int(np.clip(np.ceil(y1), top + 1, height))
    cropped = rgb[top:bottom, left:right]
    return np.asarray(
        Image.fromarray(cropped).resize((output_width, output_height), Image.Resampling.BILINEAR)
    )


def render_asymmetric_eye(
    env,
    *,
    camera_name: str,
    width: int,
    height: int,
    eye_fov: tuple[float, float, float, float],
    frustum_center_sign: float = 1.0,
) -> np.ndarray:
    """Render one MuJoCo camera with a Quest-style asymmetric frustum."""

    ctx = env.sim._render_context_offscreen
    if width > ctx.con.offWidth or height > ctx.con.offHeight:
        new_width = max(width, ctx.model.vis.global_.offwidth)
        new_height = max(height, ctx.model.vis.global_.offheight)
        ctx.update_offscreen_size(new_width, new_height)

    camera_id = env.sim.model.camera_name2id(camera_name)
    ctx.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
    ctx.cam.fixedcamid = camera_id
    mujoco.mjv_updateScene(
        ctx.model._model,
        ctx.data._data,
        ctx.vopt,
        ctx.pert,
        ctx.cam,
        mujoco.mjtCatBit.mjCAT_ALL,
        ctx.scn,
    )

    angle_left, angle_right, angle_up, angle_down = eye_fov
    gl_camera = ctx.scn.camera[0]
    near = gl_camera.frustum_near
    gl_camera.frustum_bottom = float(near * np.tan(angle_down))
    gl_camera.frustum_top = float(near * np.tan(angle_up))
    gl_camera.frustum_center = float(
        frustum_center_sign * near * (np.tan(angle_left) + np.tan(angle_right)) * 0.5
    )
    gl_camera.frustum_width = float(near * (np.tan(angle_right) - np.tan(angle_left)))

    viewport = mujoco.MjrRect(0, 0, width, height)
    mujoco.mjr_render(viewport=viewport, scn=ctx.scn, con=ctx.con)
    rgb = ctx.read_pixels(width, height, depth=False)
    return np.asarray(rgb[::-1]).copy()


class StereoPacketizer:
    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.left_encoder = None
        self.right_encoder = None
        self.pending_left_au = None
        self.pending_right_au = None
        self.sent_h264_frames = 0
        if args.codec == "h264":
            self.left_encoder = H264Encoder(
                width=args.width,
                height=args.height,
                fps=args.fps,
                label="left",
                encoder=args.h264_encoder,
                bitrate=args.h264_bitrate,
                gop=args.h264_gop,
                vaapi_device=args.vaapi_device,
            )
            self.right_encoder = H264Encoder(
                width=args.width,
                height=args.height,
                fps=args.fps,
                label="right",
                encoder=args.h264_encoder,
                bitrate=args.h264_bitrate,
                gop=args.h264_gop,
                vaapi_device=args.vaapi_device,
            )

    def close(self) -> None:
        if self.left_encoder is not None:
            self.left_encoder.close()
        if self.right_encoder is not None:
            self.right_encoder.close()

    def make_packet(self, frame: StereoRgbFrame) -> tuple[bytes, str] | None:
        left_rgb = frame.left
        right_rgb = frame.right
        if self.args.codec == "raw":
            if self.args.swap_eyes:
                left_rgb, right_rgb = right_rgb, left_rgb
            if not self.args.no_flip_y:
                left_rgb = left_rgb[::-1]
                right_rgb = right_rgb[::-1]
            payload = make_raw_stereo_payload(
                frame_index=frame.frame_index,
                width=self.args.width,
                height=self.args.height,
                left=rgb_to_rgba_bytes(left_rgb),
                right=rgb_to_rgba_bytes(right_rgb),
            )
            if len(payload) > MAX_QUEST_PAYLOAD_BYTES:
                raise ValueError(
                    f"raw stereo payload is {len(payload)} bytes; current Quest guard is "
                    f"{MAX_QUEST_PAYLOAD_BYTES}. Lower --width/--height or add H.264 encoding."
                )
            return payload, f"raw stereo frame={frame.frame_index}"

        if self.args.swap_eyes:
            left_rgb, right_rgb = right_rgb, left_rgb
        if not self.args.no_flip_y:
            left_rgb = left_rgb[::-1]
            right_rgb = right_rgb[::-1]
        if self.left_encoder is None or self.right_encoder is None:
            raise RuntimeError("H.264 encoders were not initialized")
        self.left_encoder.encode(left_rgb)
        self.right_encoder.encode(right_rgb)
        if self.pending_left_au is None:
            self.pending_left_au = self.left_encoder.take(timeout=0.02)
        if self.pending_right_au is None:
            self.pending_right_au = self.right_encoder.take(timeout=0.02)
        if self.pending_left_au is None or self.pending_right_au is None:
            return None

        left_index, left_h264 = self.pending_left_au
        right_index, right_h264 = self.pending_right_au
        h264_index = min(left_index, right_index)
        payload = make_h264_stereo_payload(
            frame_index=h264_index,
            width=self.args.width,
            height=self.args.height,
            left_h264=left_h264,
            right_h264=right_h264,
        )
        self.pending_left_au = None
        self.pending_right_au = None
        self.sent_h264_frames += 1
        return (
            payload,
            f"h264/{self.args.h264_encoder} stereo frame={h264_index} left_idx={left_index} right_idx={right_index} "
            f"bytes=({len(left_h264)},{len(right_h264)})",
        )


class RobosuiteStereoSource:
    def __init__(
        self,
        *,
        env,
        stereo,
        args: argparse.Namespace,
        left_fov: tuple[float, float, float, float],
        right_fov: tuple[float, float, float, float],
        latest_head_pose: LatestHeadPose | None = None,
        neutral_frame: QuestNeutralFrame | None = None,
    ):
        self.env = env
        self.stereo = stereo
        self.args = args
        self.left_fov = left_fov
        self.right_fov = right_fov
        self.frame_index = 0
        self.latest_head_pose = latest_head_pose
        self.neutral_frame = neutral_frame
        self.center = np.asarray([args.center_x, args.center_y, args.center_z], dtype=float)
        self.base_target = np.asarray([args.target_x, args.target_y, args.target_z], dtype=float)
        self.up = np.asarray([0.0, 0.0, 1.0], dtype=float)
        self.base_direction = self.base_target - self.center
        self.base_forward = self.base_direction / max(np.linalg.norm(self.base_direction), 1.0e-9)
        self.base_right = np.cross(self.base_direction, self.up)
        self.base_right = self.base_right / max(np.linalg.norm(self.base_right), 1.0e-9)
        self.base_up = np.cross(self.base_right, self.base_forward)
        self.base_up = self.base_up / max(np.linalg.norm(self.base_up), 1.0e-9)
        self.neutral_head_quat: np.ndarray | None = None
        self.neutral_head_position: np.ndarray | None = None
        self.neutral_head_right: np.ndarray | None = None
        self.neutral_head_up: np.ndarray | None = None
        self.neutral_head_forward: np.ndarray | None = None
        self.filtered_yaw = 0.0
        self.filtered_pitch = 0.0
        self.filtered_center_offset = np.zeros(3, dtype=float)
        self.reset_button_was_down = False

    def update_head_tracked_camera(self) -> None:
        if self.latest_head_pose is None:
            return
        pose = self.latest_head_pose.get()
        if pose is None:
            return

        current = quat_normalized_xyzw(pose)
        current_position = np.asarray([pose.px, pose.py, pose.pz], dtype=float)
        reset_pressed = pose.reset_requested and not self.reset_button_was_down
        self.reset_button_was_down = pose.reset_requested
        if self.neutral_head_quat is None or reset_pressed:
            self.neutral_head_quat = current
            self.neutral_head_position = current_position
            self.neutral_head_right = quat_rotate_xyzw(current, np.asarray([1.0, 0.0, 0.0], dtype=float))
            self.neutral_head_up = quat_rotate_xyzw(current, np.asarray([0.0, 1.0, 0.0], dtype=float))
            self.neutral_head_forward = quat_rotate_xyzw(current, np.asarray([0.0, 0.0, -1.0], dtype=float))
            if self.neutral_frame is not None:
                self.neutral_frame.set_axes(
                    self.neutral_head_right,
                    self.neutral_head_up,
                    self.neutral_head_forward,
                )
            self.filtered_yaw = 0.0
            self.filtered_pitch = 0.0
            self.filtered_center_offset.fill(0.0)
            print("head tracking neutral set from headset frame")
            return

        relative = quat_multiply_xyzw(quat_conjugate_xyzw(self.neutral_head_quat), current)
        quest_forward = quat_rotate_xyzw(relative, np.asarray([0.0, 0.0, -1.0], dtype=float))
        yaw = -np.arctan2(quest_forward[0], -quest_forward[2]) * self.args.head_yaw_gain
        pitch = np.arcsin(float(np.clip(quest_forward[1], -1.0, 1.0))) * self.args.head_pitch_gain
        yaw = float(np.clip(yaw, -np.radians(self.args.max_head_yaw_deg), np.radians(self.args.max_head_yaw_deg)))
        pitch = float(
            np.clip(pitch, -np.radians(self.args.max_head_pitch_deg), np.radians(self.args.max_head_pitch_deg))
        )
        smoothing = float(np.clip(self.args.head_smoothing, 0.0, 0.95))
        self.filtered_yaw = smoothing * self.filtered_yaw + (1.0 - smoothing) * yaw
        self.filtered_pitch = smoothing * self.filtered_pitch + (1.0 - smoothing) * pitch

        direction = rotate_axis_angle(self.base_direction, self.up, self.filtered_yaw)
        pitch_axis = np.cross(direction, self.up)
        direction = rotate_axis_angle(direction, pitch_axis, self.filtered_pitch)

        position_delta = current_position - self.neutral_head_position
        if (
            self.neutral_head_right is None
            or self.neutral_head_up is None
            or self.neutral_head_forward is None
        ):
            return
        position_delta = np.asarray(
            [
                np.dot(position_delta, self.neutral_head_right),
                np.dot(position_delta, self.neutral_head_up),
                np.dot(position_delta, self.neutral_head_forward),
            ],
            dtype=float,
        )
        position_norm = np.linalg.norm(position_delta)
        if position_norm < self.args.head_position_deadzone_m:
            position_delta = np.zeros(3, dtype=float)
        elif position_norm > 1.0e-9:
            position_delta = (
                position_delta
                * (position_norm - self.args.head_position_deadzone_m)
                / position_norm
            )
        center_offset = self.args.head_position_gain * (
            self.base_right * position_delta[0]
            + self.base_up * position_delta[1]
            + self.base_forward * position_delta[2]
        )
        self.filtered_center_offset = (
            smoothing * self.filtered_center_offset + (1.0 - smoothing) * center_offset
        )
        center = self.center + self.filtered_center_offset
        target = center + direction
        self.stereo.set_look_at(self.env, center=center, target=target, up=self.up)

    def render(self) -> StereoRgbFrame:
        self.update_head_tracked_camera()
        if self.args.projection_mode == "asymmetric":
            left_rgb = render_asymmetric_eye(
                self.env,
                camera_name=self.args.left_camera_name,
                width=self.args.width,
                height=self.args.height,
                eye_fov=self.left_fov,
                frustum_center_sign=self.args.frustum_center_sign,
            )
            right_rgb = render_asymmetric_eye(
                self.env,
                camera_name=self.args.right_camera_name,
                width=self.args.width,
                height=self.args.height,
                eye_fov=self.right_fov,
                frustum_center_sign=self.args.frustum_center_sign,
            )
        else:
            frames = self.stereo.render(self.env)
            left_rgb = frames.left.rgb
            right_rgb = frames.right.rgb

        if self.args.projection_mode == "crop":
            left_rgb = crop_to_eye_fov(
                left_rgb,
                eye_fov=self.left_fov,
                camera_fovy_deg=self.args.fovy,
                output_width=self.args.width,
                output_height=self.args.height,
            )
            right_rgb = crop_to_eye_fov(
                right_rgb,
                eye_fov=self.right_fov,
                camera_fovy_deg=self.args.fovy,
                output_width=self.args.width,
                output_height=self.args.height,
            )

        frame = StereoRgbFrame(frame_index=self.frame_index, left=left_rgb, right=right_rgb)
        self.frame_index += 1
        return frame


def make_env(args: argparse.Namespace):
    from robocorpus.robosuite.collector import RobosuiteEnvSpec, make_robosuite_env

    return make_robosuite_env(
        RobosuiteEnvSpec(
            env_name=args.env_name,
            robots=args.robot,
            controller=args.controller,
            has_renderer=False,
            has_offscreen_renderer=True,
            use_camera_obs=False,
            reward_shaping=True,
            ignore_done=True,
        )
    )


def make_stereo_camera(args: argparse.Namespace):
    from robocorpus.robosuite.cameras import SideBySideStereoCamera

    return SideBySideStereoCamera(
        left_camera_name=args.left_camera_name,
        right_camera_name=args.right_camera_name,
        baseline_m=args.baseline_m,
        width=args.render_width,
        height=args.render_height,
        depth=False,
        fovy=args.fovy,
        convergence_px=args.convergence_px,
    )


def run(args: argparse.Namespace) -> int:
    os.environ.setdefault("MUJOCO_GL", args.mujoco_gl)

    env_handle = make_env(args)
    env = env_handle.env
    stop = threading.Event()
    try:
        if args.projection_mode in {"asymmetric", "crop"}:
            left_fov = (args.left_fov_left, args.left_fov_right, args.left_fov_up, args.left_fov_down)
            right_fov = (args.right_fov_left, args.right_fov_right, args.right_fov_up, args.right_fov_down)
            args.fovy = cover_fovy_from_eye_fovs(left_fov, right_fov)
        else:
            left_fov = QUEST_LEFT_FOV
            right_fov = QUEST_RIGHT_FOV
        args.render_width = args.render_width or args.width
        args.render_height = args.render_height or args.height

        env.reset()
        stereo = make_stereo_camera(args)
        center = np.asarray([args.center_x, args.center_y, args.center_z], dtype=float)
        target = np.asarray([args.target_x, args.target_y, args.target_z], dtype=float)
        up = np.asarray([0.0, 0.0, 1.0], dtype=float)
        stereo.set_look_at(env, center=center, target=target, up=up)
        if args.baseline_axis:
            axis = np.asarray([float(part) for part in args.baseline_axis.split(",")], dtype=float)
            if axis.shape != (3,):
                raise ValueError("--baseline-axis must be three comma-separated floats")
            left_camera_id = env.sim.model.camera_name2id(args.left_camera_name)
            quat = env.sim.model.cam_quat[left_camera_id].copy()
            stereo.set_pose_with_world_axis(env, center=center, quat_wxyz=quat, axis=axis)
        left_pos, right_pos = stereo.camera_positions(env)
        print(f"stereo baseline_m={np.linalg.norm(right_pos - left_pos):.4f}")
        print(f"left_pos={left_pos.round(4).tolist()} right_pos={right_pos.round(4).tolist()}")
        print(
            f"fovy={args.fovy:.3f} render={args.render_width}x{args.render_height} "
            f"output={args.width}x{args.height} fps={args.fps} codec={args.codec}"
        )
        print(f"projection_mode={args.projection_mode} left_fov={left_fov} right_fov={right_fov}")

        neutral_frame = QuestNeutralFrame()
        latest_head_pose = LatestHeadPose() if args.head_tracking else None
        latest_quest_input = LatestQuestInput() if args.teleop_arm else None
        arm_teleop = (
            QuestArmTeleop(args, latest_quest_input, neutral_frame)
            if latest_quest_input is not None
            else None
        )
        source = RobosuiteStereoSource(
            env=env,
            stereo=stereo,
            args=args,
            left_fov=left_fov,
            right_fov=right_fov,
            latest_head_pose=latest_head_pose,
            neutral_frame=neutral_frame,
        )
        packetizer = StereoPacketizer(args)
        if args.head_tracking:
            print(
                "head_tracking=1 "
                f"yaw_gain={args.head_yaw_gain} pitch_gain={args.head_pitch_gain} "
                f"position_gain={args.head_position_gain} "
                f"position_deadzone_m={args.head_position_deadzone_m} "
                f"smoothing={args.head_smoothing} "
                f"limits=({args.max_head_yaw_deg},{args.max_head_pitch_deg})deg"
                " reset=A/X"
            )
        if args.teleop_arm:
            print(
                "teleop_arm=1 right_hand=grip_pose "
                f"clutch=right_squeeze>={args.arm_clutch_threshold} "
                f"trigger_close>={args.arm_gripper_threshold} "
                f"position_gain={args.arm_position_gain} "
                f"rotation_gain={args.arm_rotation_gain} "
                f"deadband_m={args.arm_position_deadband_m} "
                f"max_action={args.arm_max_action} "
                f"axis_signs=(forward={args.arm_forward_sign},right={args.arm_right_sign},up={args.arm_up_sign}) "
                f"lock_rotation={args.arm_lock_rotation} "
                "reset=B/Y"
            )
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind((args.host, args.port))
            server.listen(1)
            print(f"listening on {args.host}:{args.port}")
            conn, addr = server.accept()
            with conn:
                conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                print(f"Quest connected from {addr}")
                reader = threading.Thread(
                    target=reader_loop,
                    args=(conn, stop, latest_head_pose, latest_quest_input),
                    daemon=True,
                )
                reader.start()
                send_seq = 1
                sent_packets = 0
                period = 1.0 / args.fps if args.fps > 0 else 0.0
                stats_count = 0
                step_ms_sum = 0.0
                render_ms_sum = 0.0
                packet_ms_sum = 0.0
                send_ms_sum = 0.0
                loop_ms_sum = 0.0
                loop_ms_max = 0.0
                while not stop.is_set() and source.frame_index < args.frames:
                    t0 = time.perf_counter()
                    t_step0 = time.perf_counter()
                    action = (
                        arm_teleop.make_action(env.action_dim)
                        if arm_teleop is not None
                        else np.zeros(env.action_dim, dtype=np.float32)
                    )
                    env.step(action)
                    t_render0 = time.perf_counter()
                    frame = source.render()
                    t_packet0 = time.perf_counter()
                    packet = packetizer.make_packet(frame)
                    t_send0 = time.perf_counter()
                    if packet is not None:
                        payload, log = packet
                        send_packet(conn, PT_VIDEO, send_seq, payload)
                        send_seq += 1
                        sent_packets += 1
                        if sent_packets == 1 or sent_packets % 30 == 0:
                            print(f"tx {log}")
                    t_done = time.perf_counter()
                    step_ms = (t_render0 - t_step0) * 1000.0
                    render_ms = (t_packet0 - t_render0) * 1000.0
                    packet_ms = (t_send0 - t_packet0) * 1000.0
                    send_ms = (t_done - t_send0) * 1000.0
                    loop_ms = (t_done - t0) * 1000.0
                    stats_count += 1
                    step_ms_sum += step_ms
                    render_ms_sum += render_ms
                    packet_ms_sum += packet_ms
                    send_ms_sum += send_ms
                    loop_ms_sum += loop_ms
                    loop_ms_max = max(loop_ms_max, loop_ms)
                    if args.profile_stats and stats_count >= args.profile_interval:
                        inv = 1.0 / stats_count
                        print(
                            "profile "
                            f"n={stats_count} "
                            f"step={step_ms_sum * inv:.1f}ms "
                            f"render={render_ms_sum * inv:.1f}ms "
                            f"packet={packet_ms_sum * inv:.1f}ms "
                            f"send={send_ms_sum * inv:.1f}ms "
                            f"loop={loop_ms_sum * inv:.1f}ms "
                            f"max={loop_ms_max:.1f}ms"
                        )
                        stats_count = 0
                        step_ms_sum = 0.0
                        render_ms_sum = 0.0
                        packet_ms_sum = 0.0
                        send_ms_sum = 0.0
                        loop_ms_sum = 0.0
                        loop_ms_max = 0.0
                    elapsed = time.perf_counter() - t0
                    if period > elapsed:
                        time.sleep(period - elapsed)
                stop.set()
    finally:
        if "packetizer" in locals():
            packetizer.close()
        env.close()
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=7777)
    parser.add_argument("--env-name", default="Lift")
    parser.add_argument("--robot", default="Panda")
    parser.add_argument("--controller", default="OSC_POSE")
    parser.add_argument("--frames", type=int, default=1000000)
    parser.add_argument("--fps", type=float, default=72.0)
    parser.add_argument("--width", type=int, default=800)
    parser.add_argument("--height", type=int, default=450)
    parser.add_argument("--codec", choices=("raw", "h264"), default="raw")
    parser.add_argument(
        "--h264-encoder",
        choices=("libx264", "h264_nvenc", "h264_vaapi", "h264_qsv", "h264_v4l2m2m"),
        default="libx264",
    )
    parser.add_argument("--h264-bitrate", default="12M")
    parser.add_argument("--h264-gop", type=int, default=0)
    parser.add_argument("--vaapi-device", default="/dev/dri/renderD128")
    parser.add_argument("--profile-stats", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--profile-interval", type=int, default=60)
    parser.add_argument("--render-width", type=int, default=0)
    parser.add_argument("--render-height", type=int, default=0)
    parser.add_argument("--baseline-m", type=float, default=0.069637)
    parser.add_argument("--fovy", type=float, default=56.0)
    parser.add_argument(
        "--projection-mode",
        choices=("asymmetric", "crop", "symmetric"),
        default="crop",
    )
    parser.add_argument("--frustum-center-sign", type=float, default=1.0)
    parser.add_argument("--left-fov-left", type=float, default=QUEST_LEFT_FOV[0])
    parser.add_argument("--left-fov-right", type=float, default=QUEST_LEFT_FOV[1])
    parser.add_argument("--left-fov-up", type=float, default=QUEST_LEFT_FOV[2])
    parser.add_argument("--left-fov-down", type=float, default=QUEST_LEFT_FOV[3])
    parser.add_argument("--right-fov-left", type=float, default=QUEST_RIGHT_FOV[0])
    parser.add_argument("--right-fov-right", type=float, default=QUEST_RIGHT_FOV[1])
    parser.add_argument("--right-fov-up", type=float, default=QUEST_RIGHT_FOV[2])
    parser.add_argument("--right-fov-down", type=float, default=QUEST_RIGHT_FOV[3])
    parser.add_argument("--convergence-px", type=int, default=0)
    parser.add_argument("--baseline-axis", default=None)
    parser.add_argument("--swap-eyes", action="store_true")
    parser.add_argument("--no-flip-y", action="store_true")
    parser.add_argument("--head-tracking", action="store_true")
    parser.add_argument("--head-yaw-gain", type=float, default=1.0)
    parser.add_argument("--head-pitch-gain", type=float, default=1.0)
    parser.add_argument("--head-position-gain", type=float, default=1.0)
    parser.add_argument("--head-position-deadzone-m", type=float, default=0.015)
    parser.add_argument("--head-smoothing", type=float, default=0.35)
    parser.add_argument("--max-head-yaw-deg", type=float, default=45.0)
    parser.add_argument("--max-head-pitch-deg", type=float, default=30.0)
    parser.add_argument("--teleop-arm", action="store_true")
    parser.add_argument("--arm-clutch-threshold", type=float, default=0.35)
    parser.add_argument("--arm-gripper-threshold", type=float, default=0.35)
    parser.add_argument("--arm-position-gain", type=float, default=6.0)
    parser.add_argument("--arm-rotation-gain", type=float, default=6.0)
    parser.add_argument("--arm-position-deadband-m", type=float, default=0.0015)
    parser.add_argument("--arm-rotation-deadband-rad", type=float, default=0.01)
    parser.add_argument("--arm-osc-position-output-max", type=float, default=0.05)
    parser.add_argument("--arm-osc-rotation-output-max", type=float, default=0.5)
    parser.add_argument("--arm-max-action", type=float, default=1.0)
    parser.add_argument("--arm-forward-sign", type=float, default=1.0)
    parser.add_argument("--arm-right-sign", type=float, default=1.0)
    parser.add_argument("--arm-up-sign", type=float, default=1.0)
    parser.add_argument("--arm-lock-rotation", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--arm-invert-gripper", action="store_true")
    parser.add_argument("--left-camera-name", default="frontview")
    parser.add_argument("--right-camera-name", default="sideview")
    parser.add_argument("--center-x", type=float, default=-0.36)
    parser.add_argument("--center-y", type=float, default=-0.15)
    parser.add_argument("--center-z", type=float, default=1.01)
    parser.add_argument("--target-x", type=float, default=0.08)
    parser.add_argument("--target-y", type=float, default=0.0)
    parser.add_argument("--target-z", type=float, default=0.83)
    parser.add_argument("--mujoco-gl", default="egl")
    args = parser.parse_args()
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())
