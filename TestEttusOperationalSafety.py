"""Hardware-free regression tests for the Stage 3E0 safety boundary."""

from pathlib import Path
import sys
import types
import unittest


# EttusRadarSource only needs this type while constructing returned dwell data.
# Supplying a small import stub keeps these configuration tests independent of
# the rest of the application and of UHD hardware/Python bindings.
if "DataTypes" not in sys.modules:
    data_types = types.ModuleType("DataTypes")

    class RawDwellData:  # pragma: no cover - not instantiated by these tests
        pass

    data_types.RawDwellData = RawDwellData
    sys.modules["DataTypes"] = data_types

from EttusRadarSource import EttusRadarSource


class TestEttusOperationalSafety(unittest.TestCase):
    def test_defaults_are_receive_only_and_atr_disabled(self):
        source = EttusRadarSource({})

        self.assertEqual(source.OperatingMode, "RECEIVE_ONLY")
        self.assertFalse(source.TimedTransmitEnabled)
        self.assertFalse(source.AtrGpioEnabled)
        self.assertEqual(source.CommandQueueDepth, 20)
        self.assertAlmostEqual(source.CommandLeadTimeSec, 0.050)

    def test_unknown_operating_mode_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "EttusOperatingMode"):
            EttusRadarSource({"EttusOperatingMode": "UNKNOWN"})

    def test_timed_tx_rx_mode_fails_closed(self):
        with self.assertRaisesRegex(RuntimeError, "not integrated"):
            EttusRadarSource({"EttusOperatingMode": "TIMED_TX_RX"})

    def test_transmit_enable_flag_fails_closed(self):
        with self.assertRaisesRegex(RuntimeError, "not integrated"):
            EttusRadarSource({"EttusTimedTransmitEnabled": True})

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

        self.assertIn('"RadarSource": "SIM"', main_text)
        self.assertIn('"EttusOperatingMode": "RECEIVE_ONLY"', main_text)
        self.assertIn('"EttusTimedTransmitEnabled": False', main_text)
        self.assertIn('"EttusAtrGpioEnabled": False', main_text)
        self.assertIn('"EttusCommandLeadTimeSec": 0.050', main_text)


if __name__ == "__main__":
    unittest.main()
