"""Host-side safety and metadata tests for Stage 3C."""

import unittest
from unittest.mock import patch

import RunEttusTimedTransmitHardwareTest as TestProgram


class FakeTimeSpec:
    def __init__(self, value):
        self.value = float(value)


class FakeUhd:
    class types:
        class TXMetadata:
            pass
        TimeSpec = FakeTimeSpec


class TestStage3CTimedTransmit(unittest.TestCase):
    def test_safety_acknowledgement_is_mandatory(self):
        args = TestProgram.parse_arguments(["--attenuation-db", "70"])
        with self.assertRaisesRegex(ValueError, "Refusing to transmit"):
            TestProgram.validate_arguments(args)

    def test_at_least_30_db_attenuation_is_required(self):
        args = TestProgram.parse_arguments([
            "--attenuation-db", "29",
            "--i-understand-rf-output-is-enabled",
        ])
        with self.assertRaisesRegex(ValueError, "at least 30 dB"):
            TestProgram.validate_arguments(args)

    def test_initial_gain_is_fixed_at_zero_db(self):
        args = TestProgram.parse_arguments([
            "--attenuation-db", "70",
            "--gain-db", "1",
            "--i-understand-rf-output-is-enabled",
        ])
        with self.assertRaisesRegex(ValueError, "fixes TX gain at 0 dB"):
            TestProgram.validate_arguments(args)

    def test_each_pulse_is_an_isolated_timed_burst(self):
        with patch.object(TestProgram, "uhd", FakeUhd):
            metadata = TestProgram.make_tx_metadata(123.25)
        self.assertTrue(metadata.has_time_spec)
        self.assertTrue(metadata.start_of_burst)
        self.assertTrue(metadata.end_of_burst)
        self.assertEqual(metadata.time_spec.value, 123.25)

    def test_known_async_event_codes_are_exact(self):
        self.assertEqual(TestProgram.EVENT_NAMES[0x01], "burst_ack")
        self.assertEqual(TestProgram.EVENT_NAMES[0x02], "underflow")
        self.assertEqual(TestProgram.EVENT_NAMES[0x08], "time_error")
        self.assertEqual(TestProgram.EVENT_NAMES[0x10], "underflow_in_packet")


if __name__ == "__main__":
    unittest.main(verbosity=2)
