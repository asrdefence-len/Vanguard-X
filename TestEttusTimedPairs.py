"""Hardware-free tests of the Stage 3E1 bounded timed-pair path."""

from dataclasses import replace
import types
import unittest

import numpy as np

import EttusRadarSource as source_module
from EttusRadarSource import EttusRadarSource
from RadarPlans import make_uniform_dwell_plan


class FakeTimeSpec:
    def __init__(self, seconds):
        self.seconds = float(seconds)

    def get_real_secs(self):
        return self.seconds


class FakeTXMetadata:
    pass


class FakeTXAsyncMetadata:
    def __init__(self):
        self.event_code = 0


class FakeRXMetadata:
    def __init__(self):
        self.error_code = 0
        self.has_time_spec = False
        self.time_spec = FakeTimeSpec(0.0)

    def strerror(self):
        return ""


class FakeStreamCommand:
    def __init__(self, mode):
        self.mode = mode
        self.num_samps = 0
        self.stream_now = False
        self.time_spec = FakeTimeSpec(0.0)


class FakeUhd:
    types = types.SimpleNamespace(
        TXMetadata=FakeTXMetadata,
        TXAsyncMetadata=FakeTXAsyncMetadata,
        RXMetadata=FakeRXMetadata,
        TimeSpec=FakeTimeSpec,
        StreamCMD=FakeStreamCommand,
        StreamMode=types.SimpleNamespace(num_done=1, stop_cont=2),
    )


class FakeWaveformLibrary:
    def __init__(self):
        self.samples = np.ones(200, dtype=np.complex64)

    def Get(self, waveform_id):
        if waveform_id != "Frank10_20MHz":
            raise ValueError(waveform_id)
        return self.samples


class FakeTxStreamer:
    def __init__(self):
        self.sends = []
        self.events = []
        self.event_code_on_send = 0x01

    def send(self, waveform, metadata, timeout):
        self.sends.append({
            "Waveform": np.asarray(waveform).copy(),
            "TimeSec": metadata.time_spec.get_real_secs(),
            "StartOfBurst": metadata.start_of_burst,
            "EndOfBurst": metadata.end_of_burst,
            "TimeoutSec": float(timeout),
        })
        self.events.append(self.event_code_on_send)
        return len(waveform)

    def recv_async_msg(self, metadata, timeout):
        del timeout
        if not self.events:
            return False
        metadata.event_code = self.events.pop(0)
        return True


class FakeRxStreamer:
    def __init__(self):
        self.commands = []
        self.receive_index = 0

    def issue_stream_cmd(self, command):
        self.commands.append(command)

    def get_max_num_samps(self):
        return 8192

    def recv(self, buffer, metadata, timeout):
        del timeout
        command = self.commands[self.receive_index]
        self.receive_index += 1
        count = int(buffer.shape[1])
        buffer[:] = 0.0
        metadata.error_code = 0
        metadata.has_time_spec = True
        metadata.time_spec = command.time_spec
        return count


class FakeUsrp:
    def get_time_now(self):
        return FakeTimeSpec(100.0)

    def get_rx_freq(self, channel):
        del channel
        return 1.0e9

    def get_rx_gain(self, channel):
        del channel
        return 10.0

    def get_rx_antenna(self, channel):
        del channel
        return "RX2"

    def get_tx_freq(self, channel):
        del channel
        return 1.0e9

    def get_tx_gain(self, channel):
        del channel
        return 0.0

    def get_tx_antenna(self, channel):
        del channel
        return "TX/RX"


def timed_config():
    return {
        "EttusOperatingMode": "TIMED_TX_RX",
        "EttusTimedTransmitEnabled": True,
        "EttusRfOutputAcknowledged": True,
        "EttusLoopbackConfirmed": True,
        "EttusExternalAttenuationDb": 30.0,
        "EttusAtrGpioEnabled": False,
        "EttusCommandLeadTimeSec": 0.050,
        "EttusCommandQueueDepth": 2,
        "EttusRxWarmupEnabled": False,
    }


class TestEttusTimedPairs(unittest.TestCase):
    def setUp(self):
        self.original_uhd = source_module.uhd
        source_module.uhd = FakeUhd

    def tearDown(self):
        source_module.uhd = self.original_uhd

    def make_source(self):
        source = EttusRadarSource(
            timed_config(),
            TheWaveformLibrary=FakeWaveformLibrary(),
        )
        source.Usrp = FakeUsrp()
        source.RxStreamer = FakeRxStreamer()
        source.TxStreamer = FakeTxStreamer()
        source._configured_sample_rate = 40.0e6
        source._receive_path_warmed = True
        source._initialised = True
        return source

    @staticmethod
    def make_plan():
        return make_uniform_dwell_plan(
            dwell_id=7,
            task_id=1,
            task_type="SEARCH",
            waveform_id="Frank10_20MHz",
            sample_rate=40.0e6,
            num_samples=512,
            num_pulses=4,
            pri_sec=1.0e-3,
            rx_start_delay_sec=6.0e-6,
        )

    def test_complete_cpi_uses_bounded_finite_pairs(self):
        source = self.make_source()
        raw = source.ExecuteDwell(self.make_plan())

        self.assertEqual(raw.IQ.shape, (4, 512))
        self.assertEqual(len(source.RxStreamer.commands), 4)
        self.assertEqual(len(source.TxStreamer.sends), 4)
        self.assertEqual(raw.Diagnostics["TransmitCommandCount"], 4)
        self.assertEqual(
            raw.Diagnostics["TransmitBurstAcknowledgementCount"],
            4,
        )
        self.assertEqual(
            raw.Diagnostics["MaximumOutstandingTimedPairs"],
            2,
        )
        self.assertFalse(raw.Diagnostics["ReceiveOnly"])
        self.assertEqual(raw.Diagnostics["TransmitEventErrors"], [])
        self.assertTrue(all(
            item["StartOfBurst"] and item["EndOfBurst"]
            for item in source.TxStreamer.sends
        ))
        self.assertTrue(all(
            item["Waveform"].shape == (200,)
            for item in source.TxStreamer.sends
        ))

        expected_tx = 100.050 + np.arange(4) * 1.0e-3
        expected_rx = expected_tx + 6.0e-6
        actual_tx = np.asarray([
            item["TimeSec"] for item in source.TxStreamer.sends
        ])
        actual_rx = np.asarray([
            command.time_spec.get_real_secs()
            for command in source.RxStreamer.commands
        ])
        np.testing.assert_allclose(actual_tx, expected_tx, atol=1.0e-12)
        np.testing.assert_allclose(actual_rx, expected_rx, atol=1.0e-12)

    def test_tx_disabled_pulse_still_receives_but_does_not_transmit(self):
        source = self.make_source()
        plan = self.make_plan()
        plan.PulsePlans[1] = replace(plan.PulsePlans[1], TxEnabled=False)

        raw = source.ExecuteDwell(plan)

        self.assertEqual(len(source.RxStreamer.commands), 4)
        self.assertEqual(len(source.TxStreamer.sends), 3)
        self.assertEqual(raw.Diagnostics["TransmitCommandCount"], 3)
        self.assertEqual(
            raw.Diagnostics["TransmitBurstAcknowledgementCount"],
            3,
        )

    def test_tx_event_failure_is_reported_after_complete_cpi_capture(self):
        source = self.make_source()
        source.TxStreamer.event_code_on_send = 0x02
        source.TxAsyncTimeoutSec = 0.001

        with self.assertRaisesRegex(RuntimeError, "after CPI capture"):
            source.ExecuteDwell(self.make_plan())

        self.assertEqual(source.RxStreamer.receive_index, 4)


if __name__ == "__main__":
    unittest.main()
