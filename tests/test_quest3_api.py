from __future__ import annotations

import unittest
import socket

import numpy as np

from robovr.quest3 import Pose3D, Quest3InputState, Quest3Server, Quest3VideoStreamer, parse_key_value_payload
from robovr.quest3.protocol import (
    MAGIC,
    VERSION,
    PacketHeader,
    PT_POSE,
    pack_packet_header,
    unpack_packet_header,
)
from robovr.quest3.state import empty_input_state, update_state_from_pose_payload


class Quest3ApiTest(unittest.TestCase):
    def test_parse_key_value_payload_semicolon_and_newline(self) -> None:
        raw = parse_key_value_payload(b"a=1;b=two\nbad_field\nc = 3 ; =ignored")
        self.assertEqual(raw["a"], "1")
        self.assertEqual(raw["b"], "two")
        self.assertEqual(raw["c"], "3")
        self.assertNotIn("bad_field", raw)
        self.assertNotIn("", raw)

    def test_packet_header_round_trip(self) -> None:
        header = PacketHeader(
            magic=MAGIC,
            version=VERSION,
            packet_type=4,
            seq=7,
            timestamp_ns=123,
            payload_size=12,
            crc32=0,
        )
        self.assertEqual(unpack_packet_header(pack_packet_header(header)), header)

    def test_state_update_from_pose_payload(self) -> None:
        raw = parse_key_value_payload(
            "head_px=1;head_py=2;head_pz=3;head_qx=0;head_qy=0;head_qz=0;head_qw=1;"
            "right_grip_flags=2;right_grip_px=4;right_grip_py=5;right_grip_pz=6;"
            "right_grip_qx=0.1;right_grip_qy=0.2;right_grip_qz=0.3;right_grip_qw=0.9;"
            "right_trigger=0.7;right_squeeze=0.8;left_trigger=0.1;left_squeeze=0.2;"
            "button_a=1;button_b=0;button_x=true;button_y=false;ipd_m=0.07;eye_view_count=2"
        )
        state = update_state_from_pose_payload(None, raw, timestamp_ns=99)
        self.assertIsInstance(state, Quest3InputState)
        self.assertIsNotNone(state.head)
        self.assertIsNotNone(state.right_grip)
        np.testing.assert_allclose(state.head.position, np.asarray([1.0, 2.0, 3.0]))
        np.testing.assert_allclose(state.right_grip.quat_xyzw, np.asarray([0.1, 0.2, 0.3, 0.9]))
        self.assertEqual(state.right_grip_flags, 2)
        self.assertTrue(state.button_a)
        self.assertFalse(state.button_b)
        self.assertTrue(state.button_x)
        self.assertFalse(state.button_y)
        self.assertEqual(state.ipd_m, 0.07)
        self.assertEqual(state.eye_view_count, 2)

    def test_missing_pose_fields_tolerated(self) -> None:
        state = update_state_from_pose_payload(None, {"button_a": "1"}, timestamp_ns=5)
        self.assertIsNone(state.head)
        self.assertIsNone(state.right_grip)
        self.assertTrue(state.button_a)

    def test_pose_shape_validation(self) -> None:
        with self.assertRaises(ValueError):
            Pose3D(position=np.zeros(2), quat_xyzw=np.zeros(4))

    def test_connection_state_without_quest(self) -> None:
        server = Quest3Server(auto_start=False)
        self.assertFalse(server.is_connected())
        self.assertIsNone(server.latest())
        state = empty_input_state(timestamp_ns=1, connected=False)
        self.assertFalse(state.connected)

    def test_packaging_smoke_import_and_construct(self) -> None:
        from robovr.quest3 import Pose3D, Quest3InputState, Quest3Server, Quest3VideoStreamer

        self.assertIsNotNone(Pose3D)
        self.assertIsNotNone(Quest3InputState)
        server = Quest3Server(auto_start=False)
        self.assertFalse(server.is_connected())
        streamer = Quest3VideoStreamer(server)
        self.assertFalse(streamer.is_connected)

    def test_video_streamer_noops_when_disconnected(self) -> None:
        server = Quest3Server(auto_start=False)
        streamer = Quest3VideoStreamer(server)
        frame = np.zeros((450, 800, 3), dtype=np.uint8)
        self.assertFalse(streamer.send(frame, frame))
        stats = streamer.stats()
        self.assertEqual(stats["frames_submitted"], 1)
        self.assertEqual(stats["frames_dropped"], 1)
        self.assertEqual(stats["frames_sent"], 0)
        streamer.close()

    def test_server_receives_pose_packet(self) -> None:
        try:
            probe = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        except PermissionError as exc:
            self.skipTest(f"socket creation not permitted in this sandbox: {exc}")
        with probe:
            try:
                probe.bind(("127.0.0.1", 0))
            except PermissionError as exc:
                self.skipTest(f"socket bind not permitted in this sandbox: {exc}")
            port = probe.getsockname()[1]

        server = Quest3Server(port=port, timeout_s=0.2)
        server.start()
        try:
            payload = (
                b"head_px=1;head_py=2;head_pz=3;head_qx=0;head_qy=0;head_qz=0;head_qw=1;"
                b"button_a=1;right_trigger=0.5"
            )
            header = PacketHeader(
                magic=MAGIC,
                version=VERSION,
                packet_type=PT_POSE,
                seq=1,
                timestamp_ns=42,
                payload_size=len(payload),
                crc32=0,
            )
            with socket.create_connection(("127.0.0.1", port), timeout=1.0) as conn:
                conn.sendall(pack_packet_header(header) + payload)
                state = server.wait_for_state(timeout_s=1.0)
            self.assertIsNotNone(state)
            assert state is not None
            self.assertTrue(state.connected)
            self.assertTrue(state.button_a)
            self.assertEqual(state.right_trigger, 0.5)
            self.assertIsNotNone(state.head)
            np.testing.assert_allclose(state.head.position, np.asarray([1.0, 2.0, 3.0]))
        finally:
            server.close()


if __name__ == "__main__":
    unittest.main()
