"""High-level Quest 3 stereo video streaming API."""

from __future__ import annotations

import queue
import subprocess
import threading
from collections import deque
from typing import Any

import numpy as np

from .protocol import CODEC_H264_ANNEXB, VIDEO_FRAME_KEYFRAME, monotonic_ns, pack_video_frame_header
from .server import Quest3Server


def _start_code_size(data: bytes | bytearray, offset: int) -> int:
    if offset + 3 <= len(data) and data[offset : offset + 3] == b"\x00\x00\x01":
        return 3
    if offset + 4 <= len(data) and data[offset : offset + 4] == b"\x00\x00\x00\x01":
        return 4
    return 0


def _find_start_code(data: bytes | bytearray, offset: int = 0) -> int:
    for i in range(offset, max(offset, len(data) - 2)):
        if _start_code_size(data, i):
            return i
    return -1


def _nal_payload(nal: bytes) -> bytes:
    size = _start_code_size(nal, 0)
    return nal[size:] if size else nal


def _rbsp_from_ebsp(ebsp: bytes) -> bytes:
    out = bytearray()
    zeros = 0
    for byte in ebsp:
        if zeros >= 2 and byte == 0x03:
            zeros = 0
            continue
        out.append(byte)
        zeros = zeros + 1 if byte == 0 else 0
    return bytes(out)


class _BitReader:
    def __init__(self, data: bytes):
        self.data = data
        self.bit_pos = 0

    def read_bit(self) -> int:
        if self.bit_pos >= len(self.data) * 8:
            raise EOFError
        byte = self.data[self.bit_pos // 8]
        bit = (byte >> (7 - (self.bit_pos % 8))) & 1
        self.bit_pos += 1
        return bit

    def read_bits(self, count: int) -> int:
        value = 0
        for _ in range(count):
            value = (value << 1) | self.read_bit()
        return value

    def read_ue(self) -> int:
        zeros = 0
        while self.read_bit() == 0:
            zeros += 1
        return 0 if zeros == 0 else (1 << zeros) - 1 + self.read_bits(zeros)


def _first_mb_in_slice(nal: bytes) -> int | None:
    payload = _nal_payload(nal)
    if len(payload) < 2:
        return None
    try:
        return _BitReader(_rbsp_from_ebsp(payload[1:])).read_ue()
    except EOFError:
        return None


def _h264_gop_frames(fps: float, override: int) -> int:
    return max(1, int(round(fps))) if override <= 0 else int(override)


def _h264_ffmpeg_command(
    *,
    width: int,
    height: int,
    fps: float,
    encoder: str,
    bitrate: str,
    gop: int,
    vaapi_device: str,
) -> list[str]:
    command = ["ffmpeg", "-hide_banner", "-loglevel", "error"]
    if encoder == "h264_vaapi":
        command += ["-init_hw_device", f"vaapi=va:{vaapi_device}", "-filter_hw_device", "va"]
    command += [
        "-f",
        "rawvideo",
        "-pix_fmt",
        "rgb24",
        "-s:v",
        f"{width}x{height}",
        "-r",
        f"{fps:.3f}",
        "-i",
        "pipe:0",
        "-an",
    ]
    if encoder == "libx264":
        command += [
            "-c:v",
            "libx264",
            "-preset",
            "ultrafast",
            "-tune",
            "zerolatency",
            "-pix_fmt",
            "yuv420p",
            "-g",
            str(gop),
            "-keyint_min",
            str(gop),
            "-bf",
            "0",
            "-x264-params",
            "repeat-headers=1:scenecut=0",
        ]
    elif encoder == "h264_nvenc":
        command += [
            "-c:v",
            "h264_nvenc",
            "-preset",
            "p1",
            "-tune",
            "ull",
            "-profile:v",
            "baseline",
            "-rc",
            "cbr",
            "-b:v",
            bitrate,
            "-maxrate",
            bitrate,
            "-bufsize",
            bitrate,
            "-g",
            str(gop),
            "-bf",
            "0",
            "-forced-idr",
            "1",
            "-zerolatency",
            "1",
            "-aud",
            "1",
        ]
    elif encoder == "h264_vaapi":
        command += [
            "-vf",
            "format=nv12,hwupload",
            "-c:v",
            "h264_vaapi",
            "-profile:v",
            "constrained_baseline",
            "-b:v",
            bitrate,
            "-maxrate",
            bitrate,
            "-bufsize",
            bitrate,
            "-g",
            str(gop),
            "-bf",
            "0",
        ]
    elif encoder == "h264_qsv":
        command += [
            "-vf",
            "format=nv12",
            "-c:v",
            "h264_qsv",
            "-preset",
            "veryfast",
            "-look_ahead",
            "0",
            "-b:v",
            bitrate,
            "-maxrate",
            bitrate,
            "-bufsize",
            bitrate,
            "-g",
            str(gop),
            "-bf",
            "0",
        ]
    elif encoder == "h264_v4l2m2m":
        command += ["-vf", "format=nv12", "-c:v", "h264_v4l2m2m", "-b:v", bitrate, "-g", str(gop), "-bf", "0"]
    else:
        raise ValueError(f"unsupported H.264 encoder: {encoder}")
    return command + ["-f", "h264", "pipe:1"]


class _H264Encoder:
    def __init__(
        self,
        *,
        width: int,
        height: int,
        fps: float,
        label: str,
        encoder: str,
        bitrate: str,
        gop: int,
        vaapi_device: str,
    ):
        self.label = label
        self.encoder = encoder
        self.process = subprocess.Popen(
            _h264_ffmpeg_command(
                width=width,
                height=height,
                fps=fps,
                encoder=encoder,
                bitrate=bitrate,
                gop=_h264_gop_frames(fps, gop),
                vaapi_device=vaapi_device,
            ),
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            bufsize=0,
        )
        self._queue: queue.Queue[tuple[int, bytes]] = queue.Queue()
        self._stop = threading.Event()
        self._current_nals: list[bytes] = []
        self._current_has_vcl = False
        self._frame_index = 0
        self._stderr_tail: deque[str] = deque(maxlen=20)
        self._reader = threading.Thread(target=self._reader_loop, daemon=True)
        self._stderr_reader = threading.Thread(target=self._stderr_loop, daemon=True)
        self._reader.start()
        self._stderr_reader.start()

    def close(self) -> None:
        self._stop.set()
        if self.process.stdin is not None:
            try:
                self.process.stdin.close()
            except BrokenPipeError:
                pass
        try:
            self.process.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            self.process.kill()

    def encode(self, rgb: np.ndarray) -> None:
        if self.process.poll() is not None:
            detail = " ".join(self._stderr_tail)
            suffix = f": {detail}" if detail else ""
            raise RuntimeError(f"{self.label} {self.encoder} ffmpeg exited with code {self.process.returncode}{suffix}")
        if self.process.stdin is None:
            raise RuntimeError(f"{self.label} ffmpeg stdin is closed")
        self.process.stdin.write(np.ascontiguousarray(rgb, dtype=np.uint8).tobytes())

    def take(self, timeout: float = 0.02) -> tuple[int, bytes] | None:
        try:
            return self._queue.get(timeout=timeout)
        except queue.Empty:
            return None

    def _emit_current(self) -> None:
        if self._current_nals and self._current_has_vcl:
            self._queue.put((self._frame_index, b"".join(self._current_nals)))
            self._frame_index += 1
        self._current_nals = []
        self._current_has_vcl = False

    def _process_nal(self, nal: bytes) -> None:
        sc_size = _start_code_size(nal, 0)
        if not sc_size or len(nal) <= sc_size:
            return
        nal_type = nal[sc_size] & 0x1F
        is_vcl = 1 <= nal_type <= 5
        starts_new_au = nal_type == 9 or (is_vcl and _first_mb_in_slice(nal) == 0)
        if starts_new_au and self._current_has_vcl:
            self._emit_current()
        self._current_nals.append(nal)
        self._current_has_vcl = self._current_has_vcl or is_vcl

    def _reader_loop(self) -> None:
        if self.process.stdout is None:
            return
        buffer = bytearray()
        while not self._stop.is_set():
            chunk = self.process.stdout.read(4096)
            if not chunk:
                break
            buffer.extend(chunk)
            while True:
                first = _find_start_code(buffer, 0)
                if first < 0:
                    buffer.clear()
                    break
                if first > 0:
                    del buffer[:first]
                second = _find_start_code(buffer, _start_code_size(buffer, 0))
                if second < 0:
                    break
                nal = bytes(buffer[:second])
                del buffer[:second]
                self._process_nal(nal)
        if buffer:
            self._process_nal(bytes(buffer))
        self._emit_current()

    def _stderr_loop(self) -> None:
        if self.process.stderr is None:
            return
        while not self._stop.is_set():
            line = self.process.stderr.readline()
            if not line:
                break
            text = line.decode("utf-8", errors="replace").strip()
            if text:
                self._stderr_tail.append(text)


def _resize_rgb_nearest(rgb: np.ndarray, width: int, height: int) -> np.ndarray:
    if rgb.shape[0] == height and rgb.shape[1] == width:
        return rgb
    y = np.linspace(0, rgb.shape[0] - 1, height).round().astype(np.intp)
    x = np.linspace(0, rgb.shape[1] - 1, width).round().astype(np.intp)
    return rgb[y[:, None], x]


def _prepare_rgb(rgb: np.ndarray, *, width: int, height: int, flip_y: bool) -> np.ndarray:
    array = np.asarray(rgb)
    if array.dtype != np.uint8 or array.ndim != 3 or array.shape[2] != 3:
        raise ValueError("Quest3VideoStreamer.send() expects uint8 RGB arrays with shape H x W x 3")
    array = _resize_rgb_nearest(array, width, height)
    if flip_y:
        array = array[::-1]
    return np.ascontiguousarray(array)


def _make_h264_stereo_payload(
    *,
    frame_index: int,
    width: int,
    height: int,
    left_h264: bytes,
    right_h264: bytes,
) -> bytes:
    header = pack_video_frame_header(
        codec=CODEC_H264_ANNEXB,
        flags=VIDEO_FRAME_KEYFRAME,
        stream_id=0,
        frame_index=frame_index,
        capture_time_ns=monotonic_ns(),
        width=width,
        height=height,
        left_payload_size=len(left_h264),
        right_payload_size=len(right_h264),
    )
    return header + left_h264 + right_h264


class Quest3VideoStreamer:
    """Send stereo RGB frames to a connected Quest 3.

    The default settings match the validated RoboVR Quest path: 800x450 per eye,
    H.264 Annex-B packets, 72 FPS target, and vertical flip before submission.
    """

    def __init__(
        self,
        server: Quest3Server,
        *,
        width: int = 800,
        height: int = 450,
        fps: float = 72.0,
        h264_encoder: str = "libx264",
        h264_bitrate: str = "12M",
        h264_gop: int = 0,
        vaapi_device: str = "/dev/dri/renderD128",
        flip_y: bool = True,
        swap_eyes: bool = False,
    ):
        self.server = server
        self.width = int(width)
        self.height = int(height)
        self.fps = float(fps)
        self.h264_encoder = h264_encoder
        self.h264_bitrate = h264_bitrate
        self.h264_gop = int(h264_gop)
        self.vaapi_device = vaapi_device
        self.flip_y = bool(flip_y)
        self.swap_eyes = bool(swap_eyes)
        self._left_encoder: _H264Encoder | None = None
        self._right_encoder: _H264Encoder | None = None
        self._pending_left: tuple[int, bytes] | None = None
        self._pending_right: tuple[int, bytes] | None = None
        self._stats: dict[str, Any] = {
            "frames_submitted": 0,
            "frames_sent": 0,
            "frames_dropped": 0,
            "encode_pending": 0,
            "last_error": None,
            "width": self.width,
            "height": self.height,
            "fps": self.fps,
            "codec": "h264",
            "h264_encoder": self.h264_encoder,
        }

    @property
    def is_connected(self) -> bool:
        return self.server.is_connected()

    def close(self) -> None:
        if self._left_encoder is not None:
            self._left_encoder.close()
        if self._right_encoder is not None:
            self._right_encoder.close()
        self._left_encoder = None
        self._right_encoder = None
        self._pending_left = None
        self._pending_right = None

    def stats(self) -> dict[str, Any]:
        out = dict(self._stats)
        out["connected"] = self.is_connected
        return out

    def send(self, left_rgb: np.ndarray, right_rgb: np.ndarray) -> bool:
        self._stats["frames_submitted"] += 1
        if not self.is_connected:
            self._stats["frames_dropped"] += 1
            return False
        try:
            self._ensure_encoders()
            left = _prepare_rgb(left_rgb, width=self.width, height=self.height, flip_y=self.flip_y)
            right = _prepare_rgb(right_rgb, width=self.width, height=self.height, flip_y=self.flip_y)
            if self.swap_eyes:
                left, right = right, left
            assert self._left_encoder is not None and self._right_encoder is not None
            self._left_encoder.encode(left)
            self._right_encoder.encode(right)
            if self._pending_left is None:
                self._pending_left = self._left_encoder.take()
            if self._pending_right is None:
                self._pending_right = self._right_encoder.take()
            if self._pending_left is None or self._pending_right is None:
                self._stats["encode_pending"] += 1
                return False
            left_index, left_h264 = self._pending_left
            right_index, right_h264 = self._pending_right
            payload = _make_h264_stereo_payload(
                frame_index=min(left_index, right_index),
                width=self.width,
                height=self.height,
                left_h264=left_h264,
                right_h264=right_h264,
            )
            self._pending_left = None
            self._pending_right = None
            if not self.server.send_video_frame(payload):
                self._stats["frames_dropped"] += 1
                return False
            self._stats["frames_sent"] += 1
            return True
        except Exception as exc:
            self._stats["frames_dropped"] += 1
            self._stats["last_error"] = str(exc)
            return False

    def _ensure_encoders(self) -> None:
        if self._left_encoder is not None and self._right_encoder is not None:
            return
        self._left_encoder = _H264Encoder(
            width=self.width,
            height=self.height,
            fps=self.fps,
            label="left",
            encoder=self.h264_encoder,
            bitrate=self.h264_bitrate,
            gop=self.h264_gop,
            vaapi_device=self.vaapi_device,
        )
        self._right_encoder = _H264Encoder(
            width=self.width,
            height=self.height,
            fps=self.fps,
            label="right",
            encoder=self.h264_encoder,
            bitrate=self.h264_bitrate,
            gop=self.h264_gop,
            vaapi_device=self.vaapi_device,
        )
