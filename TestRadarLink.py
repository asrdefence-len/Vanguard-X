import socket
import threading
import time
import unittest
from types import SimpleNamespace

import numpy as np

from RadarLinkClient import RadarLinkClient
from RadarLinkProtocol import EncodeMessage, FrameDecoder, MakeMessage
from RadarRemoteDisplay import BuildDisplaySnapshot, RadarRemoteDisplay


class RadarLinkProtocolTests(unittest.TestCase):
    def test_fragmented_and_multiple_frames(self):
        first = EncodeMessage(MakeMessage("heartbeat", {}, 1))
        second = EncodeMessage(MakeMessage("control_state", {"DisplayMode": "STOP"}, 2))
        decoder = FrameDecoder()
        self.assertEqual(decoder.Feed(first[:3]), [])
        messages = decoder.Feed(first[3:] + second)
        self.assertEqual([item["type"] for item in messages], ["heartbeat", "control_state"])

    def test_snapshot_is_display_only_and_bounded(self):
        magnitude = np.full((4, 4000), -90.0)
        magnitude[:, 1234] = -20.0
        processed = SimpleNamespace(
            MagnitudeDb=magnitude,
            RangeAxisM=np.arange(4000, dtype=float),
            DopplerAxisHz=np.array([-2.0, -1.0, 0.0, 1.0]),
            Diagnostics={"BoresightDeg": 12.5},
        )
        snapshot = BuildDisplaySnapshot(
            {"RangeProfileDopplerMode": "MAX", "RadarLinkMaxRangeProfilePoints": 500},
            processed, [], [], [],
        )
        self.assertNotIn("Raw", snapshot["processed"])
        self.assertLessEqual(len(snapshot["processed"]["RangeAxisM"]), 500)
        self.assertAlmostEqual(max(snapshot["processed"]["DisplayRangeProfileDb"]), -20.0)


class RadarLinkRoundTripTests(unittest.TestCase):
    def test_snapshot_is_prepared_on_update_calling_thread(self):
        config = {
            "RadarLinkHost": "127.0.0.1",
            "RadarLinkPort": 0,
            "SystemMode": "SIM",
        }
        server = RadarRemoteDisplay(config)
        try:
            calling_thread = threading.current_thread()

            class CallingThreadOnlyProcessed:
                Diagnostics = {}
                RangeAxisM = np.arange(20, dtype=float)
                DopplerAxisHz = np.array([-1.0, 0.0])

                @property
                def MagnitudeDb(self):
                    if threading.current_thread() is not calling_thread:
                        raise AssertionError(
                            "processed radar arrays accessed by link thread"
                        )
                    return np.full((2, 20), -80.0)

            server.Update(
                CallingThreadOnlyProcessed(),
                [],
                Tracks=[],
                Plots=[],
            )
            with server._snapshot_lock:
                self.assertIsInstance(server._pending_snapshot_frame, bytes)
        finally:
            server.Shutdown()

    def test_control_round_trip_and_disconnect_fail_stop(self):
        config = {
            "RadarLinkHost": "127.0.0.1",
            "RadarLinkPort": 0,
            "RadarLinkHeartbeatTimeoutSec": 0.5,
            "RfTransmitAvailable": True,
            "SystemMode": "SIM",
        }
        server = RadarRemoteDisplay(config)
        deadline = time.monotonic() + 2.0
        while server.BoundPort == 0 and time.monotonic() < deadline:
            time.sleep(0.01)
        client = RadarLinkClient("127.0.0.1", server.BoundPort, heartbeat_interval_sec=0.1)
        client.Start()
        client.WaitForDisplayConfig(2.0)
        client.SendControlState({
            "DisplayMode": "SCAN",
            "ScanEnabled": True,
            "TransmitEnabled": True,
        })
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            if server.GetControlState()["DisplayMode"] == "SCAN":
                break
            time.sleep(0.01)
        state = server.GetControlState()
        self.assertEqual(state["DisplayMode"], "SCAN")
        self.assertTrue(state["TransmitEnabled"])

        processed = SimpleNamespace(
            MagnitudeDb=np.full((2, 20), -80.0),
            RangeAxisM=np.arange(20, dtype=float),
            DopplerAxisHz=np.array([-1.0, 0.0]),
            Diagnostics={"BoresightDeg": 44.0},
        )
        server.Update(processed, [], Tracks=[], Plots=[])
        deadline = time.monotonic() + 2.0
        received_snapshot = False
        while time.monotonic() < deadline and not received_snapshot:
            received_snapshot = any(
                message["type"] == "radar_snapshot"
                for message in client.DrainMessages()
            )
            time.sleep(0.01)
        self.assertTrue(received_snapshot)

        client.Close()
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            state = server.GetControlState()
            if state["DisplayMode"] == "STOP":
                break
            time.sleep(0.01)
        self.assertEqual(state["DisplayMode"], "STOP")
        self.assertFalse(state["TransmitEnabled"])
        server.Shutdown()

    def test_timing_guard_defers_prepared_snapshot_transport(self):
        config = {
            "RadarLinkHost": "127.0.0.1",
            "RadarLinkPort": 0,
            "RadarLinkHeartbeatTimeoutSec": 1.0,
            "RfTransmitAvailable": True,
            "SystemMode": "SIM",
        }
        server = RadarRemoteDisplay(config)
        client = RadarLinkClient(
            "127.0.0.1",
            server.BoundPort,
            heartbeat_interval_sec=0.1,
        )
        timing_guard_active = False
        try:
            client.Start()
            client.WaitForDisplayConfig(2.0)
            server.BeginRadarTimingCritical()
            timing_guard_active = True
            processed = SimpleNamespace(
                MagnitudeDb=np.full((2, 20), -80.0),
                RangeAxisM=np.arange(20, dtype=float),
                DopplerAxisHz=np.array([-1.0, 0.0]),
                Diagnostics={},
            )
            server.Update(processed, [], Tracks=[], Plots=[])
            time.sleep(0.15)
            self.assertFalse(any(
                message["type"] == "radar_snapshot"
                for message in client.DrainMessages()
            ))

            server.EndRadarTimingCritical()
            timing_guard_active = False
            deadline = time.monotonic() + 2.0
            received_snapshot = False
            while time.monotonic() < deadline and not received_snapshot:
                received_snapshot = any(
                    message["type"] == "radar_snapshot"
                    for message in client.DrainMessages()
                )
                time.sleep(0.01)
            self.assertTrue(received_snapshot)
        finally:
            if timing_guard_active:
                server.EndRadarTimingCritical()
            client.Close()
            server.Shutdown()


if __name__ == "__main__":
    unittest.main()
