"""Hardware-free regression tests for the Stage 3E1 safety boundary."""

from pathlib import Path
import unittest

from EttusRadarSource import EttusRadarSource


class TestEttusOperationalSafety(unittest.TestCase):
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

    def test_stage3e1_timed_mode_rejects_atr(self):
        with self.assertRaisesRegex(RuntimeError, "keeps ATR disabled"):
            EttusRadarSource({
                "EttusOperatingMode": "TIMED_TX_RX",
                "EttusTimedTransmitEnabled": True,
                "EttusRfOutputAcknowledged": True,
                "EttusLoopbackConfirmed": True,
                "EttusExternalAttenuationDb": 30.0,
                "EttusAtrGpioEnabled": True,
            })

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


if __name__ == "__main__":
    unittest.main()
