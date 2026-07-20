"""Hardware-free regression tests for Stage 3H ATR loopback integration."""

from pathlib import Path
from types import SimpleNamespace
import unittest

import numpy as np

import EttusRadarSource as source_module
from EttusOperatingProfiles import (
    ApplyOperatingProfile,
    ParseOperatingProfileArguments,
)
from EttusRadarSource import EttusRadarSource
from RadarTiming import CalculateRadarTiming


class FakeUsrp:
    def __init__(self):
        self.registers = {
            name: 0
            for name in (
                "OUT",
                "CTRL",
                "DDR",
                "ATR_0X",
                "ATR_RX",
                "ATR_TX",
                "ATR_XX",
            )
        }
        self.writes = []

    def get_gpio_banks(self, _mboard):
        return ["FP0"]

    def set_gpio_attr(self, bank, attribute, value, mask, mboard):
        if bank != "FP0" or mask != 0x0E or mboard != 0:
            raise AssertionError("wrong Stage 3H GPIO write target")
        current = self.registers[attribute]
        self.registers[attribute] = (
            (current & ~mask) | (int(value) & mask)
        )
        self.writes.append((attribute, int(value) & mask))

    def get_gpio_attr(self, bank, attribute, mboard):
        if bank != "FP0" or mboard != 0:
            raise AssertionError("wrong Stage 3H GPIO read target")
        return self.registers[attribute]


class FakeTxStreamer:
    def __init__(self):
        self.sent = None

    def send(self, waveform, _metadata, _timeout):
        self.sent = np.asarray(waveform).copy()
        return int(self.sent.size)


class FakeTxMetadata:
    pass


class FakeTimeSpec:
    def __init__(self, value):
        self.value = float(value)


class TestEttusAtrOperationalSafety(unittest.TestCase):
    @staticmethod
    def stage3h_config(**overrides):
        config = {
            "EttusOperatingMode": "TIMED_TX_RX",
            "EttusTimedTransmitEnabled": True,
            "EttusRfOutputAcknowledged": True,
            "EttusLoopbackConfirmed": True,
            "EttusExternalAttenuationDb": 30.0,
            "EttusAtrGpioEnabled": True,
            "EttusAtrCroVerifiedAcknowledged": True,
            "EttusTrmPaDisconnectedConfirmed": True,
            "EttusGPIOBank": "FP0",
            "EttusTxAtrGPIO": 1,
            "EttusRxAtrGPIO": 2,
            "EttusAtrOverlapGPIO": 3,
            "EttusTxLeadingZeroSamples": 8,
        }
        config.update(overrides)
        return config

    @staticmethod
    def timing():
        return CalculateRadarTiming({
            "WaveformId": "Frank10_20MHz",
            "SampleRateHz": 40.0e6,
            "ChipRateHz": 20.0e6,
            "ChipCount": 100,
            "SamplesPerChip": 2,
            "NumSamples": 200,
            "PulseDurationSec": 5.0e-6,
        })

    def test_profile_enables_timed_tx_rx_and_atr(self):
        args = ParseOperatingProfileArguments([
            "--stage3h-atr-loopback",
            "--attenuation-db", "30",
            "--i-understand-rf-output-is-enabled",
            "--i-confirm-txrx-to-rx2-loopback",
            "--i-confirm-trm-pa-disconnected",
            "--i-confirm-atr-cro-verified",
        ])
        config = {
            "EttusOperatingMode": "RECEIVE_ONLY",
            "EttusTimedTransmitEnabled": False,
            "EttusAtrGpioEnabled": False,
            "EttusRxFrequencyHz": 1.0e9,
            "EttusRxChannel": 0,
        }

        profile = ApplyOperatingProfile(config, args)

        self.assertEqual(profile, "STAGE3H_ATR_LOOPBACK")
        self.assertEqual(config["EttusOperatingMode"], "TIMED_TX_RX")
        self.assertTrue(config["EttusTimedTransmitEnabled"])
        self.assertTrue(config["EttusAtrGpioEnabled"])
        self.assertEqual(config["EttusTxLeadingZeroSamples"], 8)
        self.assertEqual(config["EttusTxGainDb"], 0.0)
        self.assertEqual(config["EttusRxGainDb"], 10.0)

    def test_profile_requires_completed_cro_verification(self):
        args = ParseOperatingProfileArguments([
            "--stage3h-atr-loopback",
            "--attenuation-db", "30",
            "--i-understand-rf-output-is-enabled",
            "--i-confirm-txrx-to-rx2-loopback",
            "--i-confirm-trm-pa-disconnected",
        ])
        with self.assertRaisesRegex(ValueError, "atr-cro-verified"):
            ApplyOperatingProfile({}, args)

    def test_simulation_overlap_profile_enables_atr_and_rf_target(self):
        args = ParseOperatingProfileArguments([
            "--stage3i-atr-rf-target-overlap",
            "--attenuation-db", "30",
            "--i-understand-rf-output-is-enabled",
            "--i-confirm-txrx-to-rx2-loopback",
            "--i-confirm-trm-pa-disconnected",
            "--i-confirm-atr-cro-verified",
        ])
        config = {
            "EttusOperatingMode": "RECEIVE_ONLY",
            "EttusTimedTransmitEnabled": False,
            "EttusAtrGpioEnabled": False,
            "EttusRxFrequencyHz": 1.0e9,
            "EttusRxChannel": 0,
        }

        profile = ApplyOperatingProfile(config, args)

        self.assertEqual(profile, "STAGE3I_ATR_RF_TARGET_OVERLAP")
        self.assertTrue(config["EttusAtrGpioEnabled"])
        self.assertTrue(config["EttusAtrAllowOverlapForSimulation"])
        self.assertTrue(config["EttusRfTargetEmulatorEnabled"])
        self.assertTrue(config["EttusRfTargetUseScenario"])
        self.assertEqual(config["EttusTxGainDb"], 50.0)
        self.assertEqual(config["EttusRxGainDb"], 30.0)

    def test_simulation_overlap_profile_allows_explicit_gain_overrides(self):
        args = ParseOperatingProfileArguments([
            "--stage3i-atr-rf-target-overlap",
            "--attenuation-db", "30",
            "--tx-gain-db", "12",
            "--rx-gain-db", "22",
            "--i-understand-rf-output-is-enabled",
            "--i-confirm-txrx-to-rx2-loopback",
            "--i-confirm-trm-pa-disconnected",
            "--i-confirm-atr-cro-verified",
        ])
        config = {
            "EttusOperatingMode": "RECEIVE_ONLY",
            "EttusTimedTransmitEnabled": False,
            "EttusRxFrequencyHz": 1.0e9,
            "EttusRxChannel": 0,
        }

        profile = ApplyOperatingProfile(config, args)

        self.assertEqual(profile, "STAGE3I_ATR_RF_TARGET_OVERLAP")
        self.assertEqual(config["EttusTxGainDb"], 12.0)
        self.assertEqual(config["EttusRxGainDb"], 22.0)

    def test_verified_mapping_is_fixed(self):
        source = EttusRadarSource(self.stage3h_config())
        self.assertEqual(source.GpioBank, "FP0")
        self.assertEqual(source.TxAtrGpioBit, 1)
        self.assertEqual(source.RxAtrGpioBit, 2)
        self.assertEqual(source.OverlapAtrGpioBit, 3)
        with self.assertRaisesRegex(RuntimeError, "verified FP0"):
            EttusRadarSource(self.stage3h_config(EttusTxAtrGPIO=0))

    def test_atr_state_words_and_control_order(self):
        source = EttusRadarSource(self.stage3h_config())
        source.Usrp = FakeUsrp()

        source._configure_atr_gpio()

        self.assertEqual(source.Usrp.registers["ATR_0X"], 0x00)
        self.assertEqual(source.Usrp.registers["ATR_RX"], 0x04)
        self.assertEqual(source.Usrp.registers["ATR_TX"], 0x02)
        self.assertEqual(source.Usrp.registers["ATR_XX"], 0x08)
        self.assertEqual(source.Usrp.writes[-1], ("CTRL", 0x0E))

    def test_simulation_overlap_drives_tx_rx_and_witness_high(self):
        source = EttusRadarSource(self.stage3h_config(
            EttusAtrAllowOverlapForSimulation=True,
            EttusRfTargetEmulatorEnabled=True,
            EttusRfTargetUseScenario=True,
        ))
        source.Usrp = FakeUsrp()

        source._configure_atr_gpio()

        self.assertEqual(source.Usrp.registers["ATR_0X"], 0x00)
        self.assertEqual(source.Usrp.registers["ATR_RX"], 0x04)
        self.assertEqual(source.Usrp.registers["ATR_TX"], 0x02)
        self.assertEqual(source.Usrp.registers["ATR_XX"], 0x0E)

    def test_overlap_permission_is_restricted_to_rf_target_emulation(self):
        with self.assertRaisesRegex(RuntimeError, "restricted"):
            EttusRadarSource(self.stage3h_config(
                EttusAtrAllowOverlapForSimulation=True,
                EttusRfTargetEmulatorEnabled=False,
            ))

    def test_safe_low_shutdown_takes_manual_control(self):
        source = EttusRadarSource(self.stage3h_config())
        source.Usrp = FakeUsrp()
        source._configure_atr_gpio()

        source._force_atr_safe_low()

        self.assertEqual(source.Usrp.registers["OUT"], 0x00)
        self.assertEqual(source.Usrp.registers["CTRL"], 0x00)
        self.assertEqual(source.Usrp.registers["DDR"], 0x0E)

    def test_transport_prepends_zeros_without_changing_library_waveform(self):
        original_uhd = source_module.uhd
        source_module.uhd = SimpleNamespace(types=SimpleNamespace(
            TXMetadata=FakeTxMetadata,
            TimeSpec=FakeTimeSpec,
        ))
        try:
            reference = np.ones(200, dtype=np.complex64)
            library = SimpleNamespace(Get=lambda _waveform_id: reference)
            source = EttusRadarSource(self.stage3h_config(), library)
            source._configured_sample_rate = 40.0e6
            source.TxStreamer = FakeTxStreamer()
            timing = self.timing()
            pulse = SimpleNamespace(
                TxEnabled=True,
                WaveformId="Frank10_20MHz",
                PriSec=timing.PriSec,
                RxStartDelaySec=timing.RxStartDelaySec,
                NumRxSamples=timing.NumRxSamples,
                AmplitudeScale=1.0,
                PhaseOffsetRad=0.0,
                FrequencyOffsetHz=0.0,
            )
            dwell = SimpleNamespace(
                PulsePlans=[pulse],
                NumSamples=timing.NumRxSamples,
                AzimuthDeg=0.0,
            )

            source._queue_transmit_for_pri(
                dwell,
                0,
                10.0,
                tx_time_offset_sec=0.0,
                rf_target={
                    "Active": False,
                    "RangeM": 0.0,
                    "RadialVelocityMps": 0.0,
                    "AmplitudeScale": 1.0,
                },
            )

            self.assertEqual(source.TxStreamer.sent.size, 208)
            np.testing.assert_array_equal(
                source.TxStreamer.sent[:8],
                np.zeros(8, dtype=np.complex64),
            )
            np.testing.assert_array_equal(
                source.TxStreamer.sent[8:],
                reference,
            )
            np.testing.assert_array_equal(
                library.Get("Frank10_20MHz"),
                reference,
            )
        finally:
            source_module.uhd = original_uhd

    def test_stage3f_target_accepts_pre_roll_without_range_bias(self):
        config = self.stage3h_config(
            EttusAtrGpioEnabled=False,
            EttusAtrCroVerifiedAcknowledged=False,
            EttusTrmPaDisconnectedConfirmed=False,
            EttusRfTargetEmulatorEnabled=True,
            EttusRfTargetUseScenario=True,
            EttusRfTargetRangeM=6000.0,
            EttusRfTargetAngleHalfWidthDeg=2.0,
            EttusLoopbackHardwareDelaySamples=166,
        )
        source = EttusRadarSource(config)
        sample_rate_hz = 40.0e6
        desired_delay_sec = 2.0 * 6000.0 / 299792458.0
        hardware_delay_sec = 166 / sample_rate_hz
        pre_roll_sec = 8 / sample_rate_hz

        transport_start_sec = source._target_tx_offset_sec(
            sample_rate_hz,
            target_range_m=6000.0,
        )
        received_rf_start_sec = (
            transport_start_sec + pre_roll_sec + hardware_delay_sec
        )

        self.assertAlmostEqual(
            received_rf_start_sec - pre_roll_sec,
            desired_delay_sec,
        )

    def test_timing_is_5p2_tx_6p2_rx_and_4043_samples(self):
        timing = self.timing()
        self.assertEqual(timing.TxLeadingZeroSamples, 8)
        self.assertEqual(timing.TxAtrEnvelopeSamples, 208)
        self.assertAlmostEqual(timing.TxAtrEnvelopeDurationSec, 5.2e-6)
        self.assertAlmostEqual(timing.RfPulseStartDelaySec, 0.2e-6)
        self.assertAlmostEqual(timing.RxStartDelaySec, 6.2e-6)
        self.assertAlmostEqual(
            timing.RxStartDelaySec - timing.TxAtrEnvelopeDurationSec,
            1.0e-6,
        )
        self.assertEqual(timing.NumRxSamples, 4043)
        self.assertAlmostEqual(
            timing.FirstRxSampleRangeOffsetM,
            299792458.0 * 6.0e-6 / 2.0,
        )

    def test_no_profile_main_configuration_remains_fail_closed(self):
        main_text = Path(__file__).with_name(
            "VanguardxMain_scheduler.py"
        ).read_text(encoding="utf-8")
        self.assertIn('"EttusOperatingMode": "RECEIVE_ONLY"', main_text)
        self.assertIn('"EttusTimedTransmitEnabled": False', main_text)
        self.assertIn('"EttusAtrGpioEnabled": False', main_text)
        self.assertIn('"STAGE3H_ATR_LOOPBACK"', main_text)


if __name__ == "__main__":
    unittest.main()
