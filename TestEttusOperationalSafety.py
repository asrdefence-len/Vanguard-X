"""Hardware-free regression tests for the Stage 3E1 safety boundary."""

from pathlib import Path
import unittest

from EttusRadarSource import EttusRadarSource
from RadarTiming import CalculateRadarTiming


class TestEttusOperationalSafety(unittest.TestCase):
    @staticmethod
    def _stage3h_config(**overrides):
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

    def test_defaults_are_receive_only_and_atr_disabled(self):
        source = EttusRadarSource({})

        self.assertEqual(source.OperatingMode, "RECEIVE_ONLY")
        self.assertFalse(source.TimedTransmitEnabled)
        self.assertFalse(source.AtrGpioEnabled)
        self.assertEqual(source.CommandQueueDepth, 20)
        self.assertAlmostEqual(source.CommandLeadTimeSec, 0.005)

    def test_unknown_operating_mode_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "EttusOperatingMode"):
            EttusRadarSource({"EttusOperatingMode": "UNKNOWN"})

    def test_timed_mode_requires_enable_flag(self):
        with self.assertRaisesRegex(ValueError, "requires"):
            EttusRadarSource({"EttusOperatingMode": "TIMED_TX_RX"})

    def test_enable_flag_is_rejected_in_receive_only_mode(self):
        with self.assertRaisesRegex(ValueError, "RECEIVE_ONLY"):
            EttusRadarSource({"EttusTimedTransmitEnabled": True})

    def test_timed_mode_requires_rf_acknowledgement(self):
        with self.assertRaisesRegex(RuntimeError, "acknowledgement"):
            EttusRadarSource({
                "EttusOperatingMode": "TIMED_TX_RX",
                "EttusTimedTransmitEnabled": True,
            })

    def test_timed_mode_requires_loopback_confirmation(self):
        with self.assertRaisesRegex(RuntimeError, "loopback"):
            EttusRadarSource({
                "EttusOperatingMode": "TIMED_TX_RX",
                "EttusTimedTransmitEnabled": True,
                "EttusRfOutputAcknowledged": True,
            })

    def test_timed_mode_requires_minimum_attenuation(self):
        with self.assertRaisesRegex(RuntimeError, "external attenuation"):
            EttusRadarSource({
                "EttusOperatingMode": "TIMED_TX_RX",
                "EttusTimedTransmitEnabled": True,
                "EttusRfOutputAcknowledged": True,
                "EttusLoopbackConfirmed": True,
                "EttusExternalAttenuationDb": 29.9,
            })

    def test_atr_requires_cro_verification_acknowledgement(self):
        with self.assertRaisesRegex(RuntimeError, "CRO-verification"):
            EttusRadarSource(self._stage3h_config(
                EttusAtrCroVerifiedAcknowledged=False,
            ))

    def test_atr_requires_trm_and_pa_disconnected(self):
        with self.assertRaisesRegex(RuntimeError, "TRM and PA"):
            EttusRadarSource(self._stage3h_config(
                EttusTrmPaDisconnectedConfirmed=False,
            ))

    def test_stage3h_accepts_verified_mapping_and_pre_roll(self):
        source = EttusRadarSource(self._stage3h_config())
        self.assertTrue(source.TimedTransmitEnabled)
        self.assertTrue(source.AtrGpioEnabled)
        self.assertEqual(source.GpioBank, "FP0")
        self.assertEqual(source.TxAtrGpioBit, 1)
        self.assertEqual(source.RxAtrGpioBit, 2)
        self.assertEqual(source.OverlapAtrGpioBit, 3)
        self.assertEqual(source.TxLeadingZeroSamples, 8)

    def test_stage3h_rejects_unverified_mapping(self):
        with self.assertRaisesRegex(RuntimeError, "verified FP0"):
            EttusRadarSource(self._stage3h_config(
                EttusTxAtrGPIO=0,
            ))

    def test_stage3h_rejects_wrong_pre_roll(self):
        with self.assertRaisesRegex(RuntimeError, "exactly eight"):
            EttusRadarSource(self._stage3h_config(
                EttusTxLeadingZeroSamples=7,
            ))

    def test_stage3e1_timed_mode_rejects_queue_above_proven_depth(self):
        with self.assertRaisesRegex(ValueError, "must not exceed 20"):
            EttusRadarSource({
                "EttusOperatingMode": "TIMED_TX_RX",
                "EttusTimedTransmitEnabled": True,
                "EttusRfOutputAcknowledged": True,
                "EttusLoopbackConfirmed": True,
                "EttusExternalAttenuationDb": 30.0,
                "EttusCommandQueueDepth": 21,
            })

    def test_nonpositive_command_lead_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "must be positive"):
            EttusRadarSource({"EttusCommandLeadTimeSec": 0.0})

    def test_receive_only_transmit_hook_queues_nothing(self):
        source = EttusRadarSource({})
        queued = source._queue_transmit_for_pri(None, 0, 0.0)
        self.assertFalse(queued)

    def test_main_configuration_is_explicitly_safe(self):
        main_text = (
            Path(__file__).with_name("VanguardxMain_scheduler.py")
            .read_text(encoding="utf-8")
        )

        self.assertIn('"RadarSource": "ETTUS"', main_text)
        self.assertIn('"EttusOperatingMode": "RECEIVE_ONLY"', main_text)
        self.assertIn('"EttusTimedTransmitEnabled": False', main_text)
        self.assertIn('"EttusAtrGpioEnabled": False', main_text)
        self.assertIn('"EttusTxLeadingZeroSamples": 8', main_text)
        self.assertIn('"STAGE3H_ATR_LOOPBACK"', main_text)
        self.assertIn('"EttusCommandLeadTimeSec": 0.005', main_text)
        self.assertIn('"RadarDwellIntervalSec": 0.10', main_text)

    def test_hardware_harness_accepts_operational_five_ms_lead(self):
        harness_text = (
            Path(__file__).with_name(
                "RunEttusOperationalTimedPairHardwareTest.py"
            ).read_text(encoding="utf-8")
        )

        self.assertIn(
            'parser.add_argument("--lead-ms", type=float, default=5.0)',
            harness_text,
        )
        self.assertIn("if args.lead_ms < 5.0:", harness_text)
        self.assertNotIn("at least 20 ms", harness_text)

    def test_atr_timing_has_eight_sample_pre_roll_and_one_us_guard(self):
        timing = CalculateRadarTiming({
            "WaveformId": "Frank10_20MHz",
            "SampleRateHz": 40.0e6,
            "ChipRateHz": 20.0e6,
            "ChipCount": 100,
            "SamplesPerChip": 2,
            "NumSamples": 200,
            "PulseDurationSec": 5.0e-6,
        })

        self.assertEqual(timing.TxLeadingZeroSamples, 8)
        self.assertEqual(timing.TxAtrEnvelopeSamples, 208)
        self.assertAlmostEqual(timing.TxLeadingZeroDurationSec, 0.2e-6)
        self.assertAlmostEqual(timing.TxAtrEnvelopeDurationSec, 5.2e-6)
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


if __name__ == "__main__":
    unittest.main()
