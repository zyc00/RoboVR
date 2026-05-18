"""Threaded Quest 3 TCP input server."""

from __future__ import annotations

import socket
import subprocess
import threading
from dataclasses import replace
from typing import Any

from .protocol import (
    MAGIC,
    MAX_QUEST_PAYLOAD_BYTES,
    PACKET_HEADER,
    PT_ERROR,
    PT_GOODBYE,
    PT_HEARTBEAT,
    PT_HELLO,
    PT_POSE,
    PT_STATS,
    PT_VIDEO,
    VERSION,
    parse_key_value_payload,
    recvall,
    send_packet,
    unpack_packet_header,
)
from .state import Quest3InputState, empty_input_state, update_state_from_pose_payload


class Quest3Server:
    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 7777,
        *,
        adb_reverse: bool = False,
        adb_path: str = "adb",
        auto_start: bool = False,
        timeout_s: float = 1.0,
    ):
        self.host = host
        self.port = int(port)
        self.adb_reverse = adb_reverse
        self.adb_path = adb_path
        self.timeout_s = float(timeout_s)

        self._lock = threading.Lock()
        self._condition = threading.Condition(self._lock)
        self._send_lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._server_socket: socket.socket | None = None
        self._conn: socket.socket | None = None
        self._send_seq = 1
        self._latest: Quest3InputState | None = None
        self._stats: dict[str, Any] = {
            "packets": 0,
            "hello": 0,
            "heartbeat": 0,
            "pose": 0,
            "stats": 0,
            "goodbye": 0,
            "error": 0,
            "disconnects": 0,
            "bad_packets": 0,
            "video_packets_sent": 0,
            "video_send_errors": 0,
            "last_error": None,
        }
        if auto_start:
            self.start()

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        if self.adb_reverse:
            subprocess.run(
                [self.adb_path, "reverse", f"tcp:{self.port}", f"tcp:{self.port}"],
                check=True,
            )
        self._thread = threading.Thread(target=self._run, name="Quest3Server", daemon=True)
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        self._close_socket("_conn")
        self._close_socket("_server_socket")
        if self._thread is not None:
            self._thread.join(timeout=min(self.timeout_s, 1.0))
        self._set_connected(False)

    def latest(self) -> Quest3InputState | None:
        with self._lock:
            return self._latest

    def wait_for_state(self, timeout_s: float | None = None) -> Quest3InputState | None:
        with self._condition:
            if self._latest is not None:
                return self._latest
            self._condition.wait(timeout=self.timeout_s if timeout_s is None else timeout_s)
            return self._latest

    def is_connected(self) -> bool:
        state = self.latest()
        return bool(state and state.connected)

    def stats(self) -> dict[str, Any]:
        with self._lock:
            return dict(self._stats)

    def send_payload(self, packet_type: int, payload: bytes) -> bool:
        with self._send_lock:
            conn = self._conn
            if conn is None or not self.is_connected():
                return False
            seq = self._send_seq
            self._send_seq += 1
            try:
                send_packet(conn, packet_type, seq, payload)
            except OSError as exc:
                self._bump("video_send_errors")
                self._record_error(exc)
                self._close_socket("_conn")
                self._set_connected(False)
                return False
        if packet_type == PT_VIDEO:
            self._bump("video_packets_sent")
        return True

    def send_video_frame(self, payload: bytes) -> bool:
        return self.send_payload(PT_VIDEO, payload)

    def _run(self) -> None:
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
                self._server_socket = server
                server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                server.settimeout(self.timeout_s)
                server.bind((self.host, self.port))
                server.listen(1)
                while not self._stop.is_set():
                    try:
                        conn, _addr = server.accept()
                    except socket.timeout:
                        continue
                    except OSError:
                        break
                    with conn:
                        conn.settimeout(None)
                        self._conn = conn
                        self._set_connected(True)
                        self._read_connection(conn)
                        self._conn = None
                        self._bump("disconnects")
                        self._set_connected(False)
        except Exception as exc:
            self._record_error(exc)
            self._set_connected(False)
        finally:
            self._server_socket = None

    def _read_connection(self, conn: socket.socket) -> None:
        while not self._stop.is_set():
            try:
                header_bytes = recvall(conn, PACKET_HEADER.size)
            except (OSError, socket.timeout):
                return
            if header_bytes is None:
                return
            try:
                header = unpack_packet_header(header_bytes)
            except ValueError as exc:
                self._record_error(exc)
                self._bump("bad_packets")
                return
            if (
                header.magic != MAGIC
                or header.version != VERSION
                or header.payload_size > MAX_QUEST_PAYLOAD_BYTES
            ):
                self._bump("bad_packets")
                self._record_error(
                    ValueError(
                        f"bad packet header magic={header.magic:x} version={header.version} size={header.payload_size}"
                    )
                )
                return
            try:
                payload = recvall(conn, header.payload_size) if header.payload_size else b""
            except (OSError, socket.timeout):
                return
            if payload is None:
                return
            self._handle_packet(header.packet_type, header.timestamp_ns, payload)

    def _handle_packet(self, packet_type: int, timestamp_ns: int, payload: bytes) -> None:
        self._bump("packets")
        if packet_type == PT_HELLO:
            self._bump("hello")
            raw = parse_key_value_payload(payload)
            self._update_raw(timestamp_ns, raw, connected=True)
        elif packet_type == PT_HEARTBEAT:
            self._bump("heartbeat")
            self._set_heartbeat(timestamp_ns)
        elif packet_type == PT_POSE:
            self._bump("pose")
            raw = parse_key_value_payload(payload)
            with self._condition:
                self._latest = update_state_from_pose_payload(
                    self._latest,
                    raw,
                    timestamp_ns=timestamp_ns,
                    connected=True,
                )
                self._condition.notify_all()
        elif packet_type == PT_STATS:
            self._bump("stats")
            raw = parse_key_value_payload(payload)
            self._update_raw(timestamp_ns, raw, connected=True)
            if raw.get("kind") == "video_ack":
                self._set_video_ack(timestamp_ns)
        elif packet_type == PT_GOODBYE:
            self._bump("goodbye")
            self._set_connected(False)
        elif packet_type == PT_ERROR:
            self._bump("error")
            self._update_raw(timestamp_ns, parse_key_value_payload(payload), connected=True)

    def _close_socket(self, attr: str) -> None:
        sock = getattr(self, attr)
        if sock is None:
            return
        try:
            sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        try:
            sock.close()
        except OSError:
            pass
        setattr(self, attr, None)

    def _bump(self, key: str) -> None:
        with self._lock:
            self._stats[key] = int(self._stats.get(key, 0)) + 1

    def _record_error(self, exc: Exception) -> None:
        with self._lock:
            self._stats["last_error"] = str(exc)

    def _state_or_empty(self, timestamp_ns: int, connected: bool) -> Quest3InputState:
        return self._latest or empty_input_state(timestamp_ns=timestamp_ns, connected=connected)

    def _set_connected(self, connected: bool) -> None:
        with self._condition:
            if self._latest is None:
                self._latest = empty_input_state(timestamp_ns=0, connected=connected)
            elif self._latest.connected != connected:
                self._latest = replace(self._latest, connected=connected)
            self._condition.notify_all()

    def _update_raw(self, timestamp_ns: int, raw: dict[str, str], *, connected: bool) -> None:
        with self._condition:
            base = self._state_or_empty(timestamp_ns, connected)
            merged = dict(base.raw)
            merged.update(raw)
            self._latest = replace(base, timestamp_ns=timestamp_ns, connected=connected, raw=merged)
            self._condition.notify_all()

    def _set_heartbeat(self, timestamp_ns: int) -> None:
        with self._condition:
            base = self._state_or_empty(timestamp_ns, True)
            self._latest = replace(
                base,
                timestamp_ns=timestamp_ns,
                connected=True,
                last_heartbeat_ns=timestamp_ns,
            )
            self._condition.notify_all()

    def _set_video_ack(self, timestamp_ns: int) -> None:
        with self._condition:
            base = self._state_or_empty(timestamp_ns, True)
            self._latest = replace(
                base,
                timestamp_ns=timestamp_ns,
                connected=True,
                last_video_ack_ns=timestamp_ns,
            )
            self._condition.notify_all()


Quest3Client = Quest3Server
