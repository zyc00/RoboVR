"""Quest 3 host protocol helpers."""

from __future__ import annotations

import re
import socket
import struct
import time
from dataclasses import dataclass


MAGIC = 0x50543351
VERSION = 1
PACKET_HEADER = struct.Struct("<IHHQqII")
VIDEO_FRAME_HEADER = struct.Struct("<HHIQqIIII")

PT_HELLO = 1
PT_CONFIG = 2
PT_VIDEO = 3
PT_POSE = 4
PT_STATS = 5
PT_HEARTBEAT = 6
PT_GOODBYE = 7
PT_ERROR = 8

CODEC_RAW_RGBA = 1
CODEC_H264_ANNEXB = 2
CODEC_H265_ANNEXB = 3
CODEC_AV1_ANNEXB = 4
VIDEO_FRAME_KEYFRAME = 1
MAX_QUEST_PAYLOAD_BYTES = 1024 * 1024


@dataclass(frozen=True)
class PacketHeader:
    magic: int
    version: int
    packet_type: int
    seq: int
    timestamp_ns: int
    payload_size: int
    crc32: int = 0


def monotonic_ns() -> int:
    return time.monotonic_ns()


def pack_packet_header(header: PacketHeader) -> bytes:
    return PACKET_HEADER.pack(
        header.magic,
        header.version,
        header.packet_type,
        header.seq,
        header.timestamp_ns,
        header.payload_size,
        header.crc32,
    )


def unpack_packet_header(data: bytes) -> PacketHeader:
    if len(data) != PACKET_HEADER.size:
        raise ValueError(f"packet header must be {PACKET_HEADER.size} bytes, got {len(data)}")
    magic, version, packet_type, seq, timestamp_ns, payload_size, crc32 = PACKET_HEADER.unpack(data)
    return PacketHeader(
        magic=magic,
        version=version,
        packet_type=packet_type,
        seq=seq,
        timestamp_ns=timestamp_ns,
        payload_size=payload_size,
        crc32=crc32,
    )


def make_packet_header(packet_type: int, seq: int, payload_size: int) -> PacketHeader:
    return PacketHeader(
        magic=MAGIC,
        version=VERSION,
        packet_type=packet_type,
        seq=seq,
        timestamp_ns=monotonic_ns(),
        payload_size=payload_size,
        crc32=0,
    )


def recvall(conn: socket.socket, size: int) -> bytes | None:
    chunks = bytearray()
    while len(chunks) < size:
        data = conn.recv(size - len(chunks))
        if not data:
            return None
        chunks.extend(data)
    return bytes(chunks)


def send_packet(conn: socket.socket, packet_type: int, seq: int, payload: bytes) -> None:
    conn.sendall(pack_packet_header(make_packet_header(packet_type, seq, len(payload))))
    conn.sendall(payload)


def parse_key_value_payload(payload: bytes | str) -> dict[str, str]:
    if isinstance(payload, bytes):
        text = payload.decode("utf-8", errors="replace")
    else:
        text = payload
    out: dict[str, str] = {}
    for field in re.split(r"[;\n]+", text):
        field = field.strip()
        if not field or "=" not in field:
            continue
        key, value = field.split("=", 1)
        key = key.strip()
        if not key:
            continue
        out[key] = value.strip()
    return out


def pack_video_frame_header(
    *,
    codec: int,
    flags: int,
    stream_id: int,
    frame_index: int,
    capture_time_ns: int,
    width: int,
    height: int,
    left_payload_size: int,
    right_payload_size: int,
) -> bytes:
    return VIDEO_FRAME_HEADER.pack(
        codec,
        flags,
        stream_id,
        frame_index,
        capture_time_ns,
        width,
        height,
        left_payload_size,
        right_payload_size,
    )
