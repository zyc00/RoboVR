"""Shared Quest streaming protocol and encoder helpers."""

from __future__ import annotations

import queue
import subprocess
import threading
from collections import deque

import numpy as np

from robovr.quest3.protocol import (
    CODEC_H264_ANNEXB,
    CODEC_RAW_RGBA,
    MAGIC,
    MAX_QUEST_PAYLOAD_BYTES,
    PACKET_HEADER,
    PT_HEARTBEAT,
    PT_HELLO,
    PT_POSE,
    PT_STATS,
    PT_VIDEO,
    VERSION,
    VIDEO_FRAME_KEYFRAME,
    monotonic_ns,
    pack_video_frame_header,
    parse_key_value_payload as parse_kv,
    recvall,
    send_packet,
)


def start_code_size(data: bytes | bytearray, offset: int) -> int:
    if offset + 3 <= len(data) and data[offset : offset + 3] == b"\x00\x00\x01":
        return 3
    if offset + 4 <= len(data) and data[offset : offset + 4] == b"\x00\x00\x00\x01":
        return 4
    return 0


def find_start_code(data: bytes | bytearray, offset: int = 0) -> int:
    for i in range(offset, max(offset, len(data) - 2)):
        if start_code_size(data, i):
            return i
    return -1


def nal_payload(nal: bytes) -> bytes:
    sc_size = start_code_size(nal, 0)
    return nal[sc_size:] if sc_size else nal


def rbsp_from_ebsp(ebsp: bytes) -> bytes:
    out = bytearray()
    zeros = 0
    for byte in ebsp:
        if zeros >= 2 and byte == 0x03:
            zeros = 0
            continue
        out.append(byte)
        if byte == 0:
            zeros += 1
        else:
            zeros = 0
    return bytes(out)


class BitReader:
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
        if zeros == 0:
            return 0
        return (1 << zeros) - 1 + self.read_bits(zeros)


def first_mb_in_slice(nal: bytes) -> int | None:
    payload = nal_payload(nal)
    if len(payload) < 2:
        return None
    rbsp = rbsp_from_ebsp(payload[1:])
    try:
        return BitReader(rbsp).read_ue()
    except EOFError:
        return None


def h264_gop_frames(fps: float, override: int) -> int:
    if override > 0:
        return override
    return max(1, int(round(fps)))


def build_h264_ffmpeg_command(
    *,
    width: int,
    height: int,
    fps: float,
    encoder: str,
    bitrate: str,
    gop: int,
    vaapi_device: str = "/dev/dri/renderD128",
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
    command += ["-f", "h264", "pipe:1"]
    return command


class H264Encoder:
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
        vaapi_device: str = "/dev/dri/renderD128",
    ):
        self.width = int(width)
        self.height = int(height)
        self.label = label
        self.encoder = encoder
        command = build_h264_ffmpeg_command(
            width=self.width,
            height=self.height,
            fps=fps,
            encoder=encoder,
            bitrate=bitrate,
            gop=h264_gop_frames(fps, gop),
            vaapi_device=vaapi_device,
        )
        self.process = subprocess.Popen(
            command,
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
        if self.process.stdin:
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

    def take(self, timeout: float = 0.25) -> tuple[int, bytes] | None:
        try:
            return self._queue.get(timeout=timeout)
        except queue.Empty:
            return None

    def _emit_current(self) -> None:
        if not self._current_nals or not self._current_has_vcl:
            self._current_nals.clear()
            self._current_has_vcl = False
            return
        self._queue.put((self._frame_index, b"".join(self._current_nals)))
        self._frame_index += 1
        self._current_nals = []
        self._current_has_vcl = False

    def _process_nal(self, nal: bytes) -> None:
        sc_size = start_code_size(nal, 0)
        if not sc_size or len(nal) <= sc_size:
            return
        nal_type = nal[sc_size] & 0x1F
        is_vcl = 1 <= nal_type <= 5
        starts_new_au = nal_type == 9
        if is_vcl:
            starts_new_au = first_mb_in_slice(nal) == 0
        if starts_new_au and self._current_has_vcl:
            self._emit_current()
        self._current_nals.append(nal)
        if is_vcl:
            self._current_has_vcl = True

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
                first = find_start_code(buffer, 0)
                if first < 0:
                    buffer.clear()
                    break
                if first > 0:
                    del buffer[:first]
                second = find_start_code(buffer, start_code_size(buffer, 0))
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


def make_raw_stereo_payload(
    *,
    frame_index: int,
    width: int,
    height: int,
    left: bytes,
    right: bytes,
) -> bytes:
    header = pack_video_frame_header(
        codec=CODEC_RAW_RGBA,
        flags=VIDEO_FRAME_KEYFRAME,
        stream_id=0,
        frame_index=frame_index,
        capture_time_ns=monotonic_ns(),
        width=width,
        height=height,
        left_payload_size=len(left),
        right_payload_size=len(right),
    )
    return header + left + right


def make_h264_stereo_payload(
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
