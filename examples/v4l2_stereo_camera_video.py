"""Stream a side-by-side V4L2 stereo camera to the Quest client.

Default target is a USB stereo camera that exposes 2560x720 MJPEG at 30 FPS:
left eye in the left half, right eye in the right half. The script encodes each
eye as H.264 Annex-B and sends the same RoboVR video packets as the robosuite
example.
"""

from __future__ import annotations

import argparse
import glob
import queue
import socket
import struct
import subprocess
import threading
import time
from collections import deque
from dataclasses import dataclass

import cv2
import numpy as np


MAGIC = 0x50543351
VERSION = 1
PACKET_HEADER = struct.Struct("<IHHQqII")
VIDEO_FRAME_HEADER = struct.Struct("<HHIQqIIII")

PT_HELLO = 1
PT_VIDEO = 3
PT_STATS = 5
PT_HEARTBEAT = 6

CODEC_H264_ANNEXB = 2
VIDEO_FRAME_KEYFRAME = 1


@dataclass
class StereoFrame:
    frame_index: int
    left_rgb: np.ndarray
    right_rgb: np.ndarray


def monotonic_ns() -> int:
    return time.monotonic_ns()


def recvall(conn: socket.socket, size: int) -> bytes | None:
    chunks = bytearray()
    while len(chunks) < size:
        data = conn.recv(size - len(chunks))
        if not data:
            return None
        chunks.extend(data)
    return bytes(chunks)


def send_packet(conn: socket.socket, packet_type: int, seq: int, payload: bytes) -> None:
    header = PACKET_HEADER.pack(MAGIC, VERSION, packet_type, seq, monotonic_ns(), len(payload), 0)
    conn.sendall(header)
    conn.sendall(payload)


def parse_kv(payload: bytes) -> dict[str, str]:
    text = payload.decode("utf-8", errors="replace")
    out: dict[str, str] = {}
    for field in text.split(";"):
        if "=" in field:
            key, value = field.split("=", 1)
            out[key] = value
    return out


def reader_loop(conn: socket.socket, stop: threading.Event) -> None:
    heartbeat_count = 0
    video_ack_count = 0
    while not stop.is_set():
        header_bytes = recvall(conn, PACKET_HEADER.size)
        if header_bytes is None:
            stop.set()
            return
        magic, version, packet_type, seq, timestamp_ns, payload_size, crc32 = PACKET_HEADER.unpack(header_bytes)
        del timestamp_ns, crc32
        if magic != MAGIC or version != VERSION or payload_size > 1024 * 1024:
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
) -> list[str]:
    command = [
        "ffmpeg",
        "-hide_banner",
        "-loglevel",
        "error",
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
    else:
        raise ValueError(f"unsupported encoder: {encoder}")
    command += ["-f", "h264", "pipe:1"]
    return command


class H264Encoder:
    def __init__(self, *, width: int, height: int, fps: float, label: str, encoder: str, bitrate: str, gop: int):
        self.width = int(width)
        self.height = int(height)
        self.label = label
        self.encoder = encoder
        self.process = subprocess.Popen(
            build_h264_ffmpeg_command(
                width=self.width,
                height=self.height,
                fps=fps,
                encoder=encoder,
                bitrate=bitrate,
                gop=h264_gop_frames(fps, gop),
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


class StereoPacketizer:
    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.left_encoder = H264Encoder(
            width=args.eye_width,
            height=args.eye_height,
            fps=args.fps,
            label="left",
            encoder=args.h264_encoder,
            bitrate=args.h264_bitrate,
            gop=args.h264_gop,
        )
        self.right_encoder = H264Encoder(
            width=args.eye_width,
            height=args.eye_height,
            fps=args.fps,
            label="right",
            encoder=args.h264_encoder,
            bitrate=args.h264_bitrate,
            gop=args.h264_gop,
        )
        self.pending_left_au = None
        self.pending_right_au = None

    def close(self) -> None:
        self.left_encoder.close()
        self.right_encoder.close()

    def make_packet(self, frame: StereoFrame) -> tuple[bytes, str] | None:
        left_rgb = frame.left_rgb
        right_rgb = frame.right_rgb
        if self.args.swap_eyes:
            left_rgb, right_rgb = right_rgb, left_rgb
        if not self.args.no_flip_y:
            left_rgb = left_rgb[::-1]
            right_rgb = right_rgb[::-1]

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
        header = VIDEO_FRAME_HEADER.pack(
            CODEC_H264_ANNEXB,
            VIDEO_FRAME_KEYFRAME,
            0,
            h264_index,
            monotonic_ns(),
            self.args.eye_width,
            self.args.eye_height,
            len(left_h264),
            len(right_h264),
        )
        self.pending_left_au = None
        self.pending_right_au = None
        payload = header + left_h264 + right_h264
        log = (
            f"h264/{self.args.h264_encoder} stereo frame={h264_index} "
            f"left_idx={left_index} right_idx={right_index} bytes=({len(left_h264)},{len(right_h264)})"
        )
        return payload, log


class V4L2StereoSource:
    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.frame_index = 0
        self.cap: cv2.VideoCapture | None = None
        self.device = ""
        self._open_camera()

    def _device_candidates(self) -> list[str]:
        candidates: list[str] = []
        for path in [self.args.device, self.args.device_by_id]:
            if path and path not in candidates:
                candidates.append(path)
        for path in sorted(glob.glob("/dev/v4l/by-id/*video-index0")):
            if path not in candidates:
                candidates.append(path)
        for path in sorted(glob.glob("/dev/video*")):
            if path not in candidates:
                candidates.append(path)
        return candidates

    def _open_camera(self) -> None:
        last_error = ""
        for device in self._device_candidates():
            cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
            if not cap.isOpened():
                cap.release()
                last_error = f"failed to open camera: {device}"
                continue
            self.cap = cap
            self.device = device
            self._configure_camera()
            return
        raise RuntimeError(last_error or "failed to open any V4L2 camera")

    def _configure_camera(self) -> None:
        if self.cap is None:
            raise RuntimeError("camera is not open")
        args = self.args
        fourcc = cv2.VideoWriter_fourcc(*args.fourcc)
        self.cap.set(cv2.CAP_PROP_FOURCC, fourcc)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.capture_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.capture_height)
        self.cap.set(cv2.CAP_PROP_FPS, args.fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, args.buffers)
        actual_fourcc = int(self.cap.get(cv2.CAP_PROP_FOURCC))
        actual_fourcc_text = "".join(chr((actual_fourcc >> (8 * i)) & 0xFF) for i in range(4))
        print(
            "camera opened "
            f"device={self.device} requested={args.fourcc}:{args.capture_width}x{args.capture_height}@{args.fps:.1f} "
            f"actual_fourcc={actual_fourcc_text!r} "
            f"actual={self.cap.get(cv2.CAP_PROP_FRAME_WIDTH):.0f}x{self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT):.0f}"
        )

    def close(self) -> None:
        if self.cap is not None:
            self.cap.release()
            self.cap = None

    def read(self) -> StereoFrame:
        for attempt in range(self.args.reopen_attempts + 1):
            if self.cap is None or not self.cap.isOpened():
                self.close()
                time.sleep(self.args.reopen_delay_s)
                self._open_camera()
            ok, bgr = self.cap.read()
            if ok and bgr is not None:
                break
            print(f"camera read failed device={self.device}; reopening attempt={attempt + 1}")
            self.close()
            time.sleep(self.args.reopen_delay_s)
        else:
            raise RuntimeError("camera read failed after reopen attempts")
        if self.args.layout == "sbs":
            height, width = bgr.shape[:2]
            half = width // 2
            left_bgr = bgr[:, :half]
            right_bgr = bgr[:, half : half * 2]
        elif self.args.layout == "mono-duplicate":
            left_bgr = bgr
            right_bgr = bgr
        else:
            raise ValueError(f"unsupported layout: {self.args.layout}")

        left_rgb = self._to_eye_rgb(left_bgr, eye="left")
        right_rgb = self._to_eye_rgb(right_bgr, eye="right")
        frame = StereoFrame(self.frame_index, left_rgb, right_rgb)
        self.frame_index += 1
        return frame

    def _to_eye_rgb(self, bgr: np.ndarray, *, eye: str) -> np.ndarray:
        bgr = self._crop_eye(bgr, eye=eye)
        bgr = self._resize_eye(bgr)
        return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

    def _resize_eye(self, bgr: np.ndarray) -> np.ndarray:
        if self.args.fit_mode == "stretch":
            if bgr.shape[1] != self.args.eye_width or bgr.shape[0] != self.args.eye_height:
                return cv2.resize(bgr, (self.args.eye_width, self.args.eye_height), interpolation=cv2.INTER_AREA)
            return bgr

        src_height, src_width = bgr.shape[:2]
        scale = min(self.args.eye_width / src_width, self.args.eye_height / src_height)
        fit_width = max(2, int(round(src_width * scale)))
        fit_height = max(2, int(round(src_height * scale)))
        resized = cv2.resize(bgr, (fit_width, fit_height), interpolation=cv2.INTER_AREA)
        out = np.zeros((self.args.eye_height, self.args.eye_width, 3), dtype=np.uint8)
        x0 = (self.args.eye_width - fit_width) // 2
        y0 = (self.args.eye_height - fit_height) // 2
        out[y0 : y0 + fit_height, x0 : x0 + fit_width] = resized
        return out

    def _crop_eye(self, bgr: np.ndarray, *, eye: str) -> np.ndarray:
        height, width = bgr.shape[:2]
        crop_scale = float(np.clip(self.args.crop_scale, 0.10, 1.0))
        crop_width = int(round(width * crop_scale))
        crop_height = int(round(crop_width / self.args.camera_aspect))
        if crop_height > height:
            crop_height = int(round(height * crop_scale))
            crop_width = int(round(crop_height * self.args.camera_aspect))
        crop_width = int(np.clip(crop_width, 2, width))
        crop_height = int(np.clip(crop_height, 2, height))
        if eye == "left":
            x_offset = self.args.left_x_offset_px + self.args.convergence_px
            y_offset = self.args.left_y_offset_px
        elif eye == "right":
            x_offset = self.args.right_x_offset_px - self.args.convergence_px
            y_offset = self.args.right_y_offset_px
        else:
            raise ValueError(f"unknown eye: {eye}")
        x_center = width * 0.5
        y_center = height * 0.5
        x0 = int(round(x_center + x_offset - crop_width * 0.5))
        y0 = int(round(y_center + y_offset - crop_height * 0.5))
        x0 = int(np.clip(x0, 0, width - crop_width))
        y0 = int(np.clip(y0, 0, height - crop_height))
        return bgr[y0 : y0 + crop_height, x0 : x0 + crop_width]


def run(args: argparse.Namespace) -> int:
    stop = threading.Event()
    source = V4L2StereoSource(args)
    packetizer = StereoPacketizer(args)
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind((args.host, args.port))
            server.listen(1)
            print(f"listening on {args.host}:{args.port}")
            conn, addr = server.accept()
            with conn:
                conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                print(f"Quest connected from {addr}")
                reader = threading.Thread(target=reader_loop, args=(conn, stop), daemon=True)
                reader.start()
                send_seq = 1
                sent_packets = 0
                stats_count = 0
                capture_ms_sum = 0.0
                packet_ms_sum = 0.0
                send_ms_sum = 0.0
                loop_ms_sum = 0.0
                loop_ms_max = 0.0
                while not stop.is_set() and source.frame_index < args.frames:
                    t0 = time.perf_counter()
                    frame = source.read()
                    t_capture = time.perf_counter()
                    packet = packetizer.make_packet(frame)
                    t_packet = time.perf_counter()
                    if packet is not None:
                        payload, log = packet
                        send_packet(conn, PT_VIDEO, send_seq, payload)
                        send_seq += 1
                        sent_packets += 1
                        if sent_packets == 1 or sent_packets % 30 == 0:
                            print(f"tx {log}")
                    t_done = time.perf_counter()

                    capture_ms = (t_capture - t0) * 1000.0
                    packet_ms = (t_packet - t_capture) * 1000.0
                    send_ms = (t_done - t_packet) * 1000.0
                    loop_ms = (t_done - t0) * 1000.0
                    stats_count += 1
                    capture_ms_sum += capture_ms
                    packet_ms_sum += packet_ms
                    send_ms_sum += send_ms
                    loop_ms_sum += loop_ms
                    loop_ms_max = max(loop_ms_max, loop_ms)
                    if args.profile_stats and stats_count >= args.profile_interval:
                        inv = 1.0 / stats_count
                        print(
                            "profile "
                            f"n={stats_count} "
                            f"capture={capture_ms_sum * inv:.1f}ms "
                            f"packet={packet_ms_sum * inv:.1f}ms "
                            f"send={send_ms_sum * inv:.1f}ms "
                            f"loop={loop_ms_sum * inv:.1f}ms "
                            f"max={loop_ms_max:.1f}ms"
                        )
                        stats_count = 0
                        capture_ms_sum = 0.0
                        packet_ms_sum = 0.0
                        send_ms_sum = 0.0
                        loop_ms_sum = 0.0
                        loop_ms_max = 0.0
                stop.set()
    finally:
        packetizer.close()
        source.close()
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=7777)
    parser.add_argument("--device", default="/dev/video0")
    parser.add_argument("--device-by-id", default="/dev/v4l/by-id/usb-USB_Camera_USB_Camera_01.00.00-video-index0")
    parser.add_argument("--capture-width", type=int, default=2560)
    parser.add_argument("--capture-height", type=int, default=720)
    parser.add_argument("--eye-width", type=int, default=960)
    parser.add_argument("--eye-height", type=int, default=1040)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--fourcc", default="MJPG")
    parser.add_argument("--layout", choices=("sbs", "mono-duplicate"), default="sbs")
    parser.add_argument("--frames", type=int, default=1000000)
    parser.add_argument("--buffers", type=int, default=1)
    parser.add_argument("--reopen-attempts", type=int, default=10)
    parser.add_argument("--reopen-delay-s", type=float, default=0.5)
    parser.add_argument("--h264-encoder", choices=("libx264", "h264_nvenc"), default="libx264")
    parser.add_argument("--h264-bitrate", default="16M")
    parser.add_argument("--h264-gop", type=int, default=0)
    parser.add_argument("--swap-eyes", action="store_true")
    parser.add_argument("--no-flip-y", action="store_true")
    parser.add_argument(
        "--crop-scale",
        type=float,
        default=0.72,
        help="Center crop per-eye camera images before resizing. Smaller values reduce edge lens distortion.",
    )
    parser.add_argument(
        "--camera-aspect",
        type=float,
        default=16.0 / 9.0,
        help="Aspect ratio to preserve from each physical camera image before Quest eye padding.",
    )
    parser.add_argument(
        "--fit-mode",
        choices=("letterbox", "stretch"),
        default="letterbox",
        help="letterbox preserves camera geometry inside the Quest eye texture; stretch fills the texture.",
    )
    parser.add_argument(
        "--convergence-px",
        type=float,
        default=0.0,
        help="Shift left crop right and right crop left by this many source pixels.",
    )
    parser.add_argument("--left-x-offset-px", type=float, default=0.0)
    parser.add_argument("--right-x-offset-px", type=float, default=0.0)
    parser.add_argument("--left-y-offset-px", type=float, default=0.0)
    parser.add_argument("--right-y-offset-px", type=float, default=0.0)
    parser.add_argument("--profile-stats", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--profile-interval", type=int, default=60)
    args = parser.parse_args()
    if len(args.fourcc) != 4:
        raise ValueError("--fourcc must be exactly four characters, e.g. MJPG")
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())
