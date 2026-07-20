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

    def make_source(self, config=None):
        source = EttusRadarSource(
            timed_config() if config is None else config,
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
    def make_plan(num_samples=512):
        return make_uniform_dwell_plan(
            dwell_id=7,
            task_id=1,
            task_type="SEARCH",
            waveform_id="Frank10_20MHz",
            sample_rate=40.0e6,
            num_samples=num_samples,
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

    def test_rf_target_delays_main_pulse_only_inside_theta_plus_minus_two(self):
        config = timed_config()
        config.update({
            "EttusRfTargetEmulatorEnabled": True,
            "EttusRfTargetRangeM": 6000.0,
            "EttusRfTargetBearingDeg": 80.0,
            "EttusRfTargetAngleHalfWidthDeg": 2.0,
            "EttusLoopbackHardwareDelaySamples": 166,
            "RfFrequency": 9.4e9,
        })

        active_source = self.make_source(config)
        active_plan = self.make_plan(num_samples=4043)
        active_plan.AzimuthDeg = 82.0
        active_raw = active_source.ExecuteDwell(active_plan)

        expected_offset = (
            2.0 * 6000.0 / 299792458.0 - 166.0 / 40.0e6
        )
        self.assertTrue(active_raw.Diagnostics["RfTargetEmulatorActive"])
        self.assertAlmostEqual(
            active_raw.Diagnostics["RfTargetTxOffsetSec"],
            expected_offset,
        )
        self.assertAlmostEqual(
            active_source.TxStreamer.sends[0]["TimeSec"],
            100.050 + expected_offset,
        )

        lower_edge_source = self.make_source(config)
        lower_edge_plan = self.make_plan(num_samples=4043)
        lower_edge_plan.AzimuthDeg = 78.0
        lower_edge_raw = lower_edge_source.ExecuteDwell(lower_edge_plan)
        self.assertTrue(lower_edge_raw.Diagnostics["RfTargetEmulatorActive"])

        inactive_source = self.make_source(config)
        inactive_plan = self.make_plan(num_samples=4043)
        inactive_plan.AzimuthDeg = 82.01
        inactive_raw = inactive_source.ExecuteDwell(inactive_plan)

        self.assertFalse(inactive_raw.Diagnostics["RfTargetEmulatorActive"])
        self.assertAlmostEqual(
            inactive_source.TxStreamer.sends[0]["TimeSec"],
            100.050,
        )

    def test_rf_target_velocity_applies_slow_time_phase(self):
        config = timed_config()
        config.update({
            "EttusRfTargetEmulatorEnabled": True,
            "EttusRfTargetRangeM": 6000.0,
            "EttusRfTargetBearingDeg": 80.0,
            "EttusRfTargetAngleHalfWidthDeg": 2.0,
            "EttusRfTargetRadialVelocityMps": 5.0,
            "EttusLoopbackHardwareDelaySamples": 166,
            "RfFrequency": 9.4e9,
        })
        source = self.make_source(config)
        plan = self.make_plan(num_samples=4043)
        plan.AzimuthDeg = 80.0

        source.ExecuteDwell(plan)

        wavelength_m = 299792458.0 / 9.4e9
        doppler_hz = 2.0 * 5.0 / wavelength_m
        expected_phase = 2.0 * np.pi * doppler_hz * 1.0e-3
        measured_phase = np.angle(
            source.TxStreamer.sends[1]["Waveform"][0]
            / source.TxStreamer.sends[0]["Waveform"][0]
        )
        self.assertAlmostEqual(
            np.angle(np.exp(1j * measured_phase)),
            np.angle(np.exp(1j * expected_phase)),
            places=5,
        )

    def test_target_scenario_drives_actual_transmit_time_and_doppler(self):
        config = timed_config()
        config.update({
            "EttusRfTargetEmulatorEnabled": True,
            "EttusRfTargetUseScenario": True,
            "EttusRfTargetAngleHalfWidthDeg": 2.0,
            "EttusRfTargetAmplitudeScale": 1.0,
            "EttusLoopbackHardwareDelaySamples": 166,
            "RfFrequency": 9.4e9,
            "SceneReturns": [{
                "name": "ScenarioShip_S01",
                "parent_name": "ScenarioShip",
                "range_m": 5000.0,
                "bearing_deg": 80.0,
                "radial_velocity_mps": 4.0,
                "amplitude": 10.0,
            }],
        })
        source = self.make_source(config)
        plan = self.make_plan(num_samples=4043)
        plan.AzimuthDeg = 80.0

        raw = source.ExecuteDwell(plan)

        expected_offset = (
            2.0 * 5000.0 / 299792458.0 - 166.0 / 40.0e6
        )
        self.assertTrue(raw.Diagnostics["RfTargetEmulatorActive"])
        self.assertTrue(raw.Diagnostics["RfTargetUseScenario"])
        self.assertEqual(raw.Diagnostics["RfTargetName"], "ScenarioShip")
        self.assertAlmostEqual(raw.Diagnostics["RfTargetRangeM"], 5000.0)
        self.assertAlmostEqual(
            source.TxStreamer.sends[0]["TimeSec"],
            100.050 + expected_offset,
        )

        wavelength_m = 299792458.0 / 9.4e9
        doppler_hz = 2.0 * 4.0 / wavelength_m
        expected_phase = 2.0 * np.pi * doppler_hz * 1.0e-3
        measured_phase = np.angle(
            source.TxStreamer.sends[1]["Waveform"][0]
            / source.TxStreamer.sends[0]["Waveform"][0]
        )
        self.assertAlmostEqual(
            np.angle(np.exp(1j * measured_phase)),
            np.angle(np.exp(1j * expected_phase)),
            places=5,
        )


if __name__ == "__main__":
    unittest.main()
