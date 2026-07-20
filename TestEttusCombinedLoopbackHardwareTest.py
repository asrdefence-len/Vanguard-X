import unittest
from unittest.mock import patch

import numpy as np

import RunEttusCombinedLoopbackHardwareTest as Program


class FakeTimeSpec:
    def __init__(self, value): self.value = float(value)


class FakeUhd:
    class types:
        class TXMetadata: pass
        TimeSpec = FakeTimeSpec


class FakeAsyncStreamer:
    def __init__(self, codes): self.codes = list(codes)
    def recv_async_msg(self, metadata, timeout):
        if not self.codes: return False
        metadata.event_code = self.codes.pop(0)
        return True


class TestCombinedLoopback(unittest.TestCase):
    def valid_args(self):
        return Program.parse_arguments([
            "--attenuation-db", "70",
            "--i-understand-rf-output-is-enabled",
            "--i-confirm-txrx-to-rx2-loopback",
        ])

    def test_two_independent_safety_acknowledgements_are_required(self):
        args = Program.parse_arguments(["--attenuation-db", "70"])
        with self.assertRaises(ValueError): Program.validate_arguments(args)
        args.i_understand_rf_output_is_enabled = True
        with self.assertRaises(ValueError): Program.validate_arguments(args)

    def test_default_diagnostic_pulse_is_inside_receive_window(self):
        args = self.valid_args()
        Program.validate_arguments(args)
        self.assertEqual(round((args.tx_test_offset_us-args.rx_start_us)*40), 160)

    def test_sband_style_tx_first_order_is_operator_selectable(self):
        args = self.valid_args()
        args.pair_order = "tx-first"
        Program.validate_arguments(args)
        self.assertEqual(args.pair_order, "tx-first")

    def test_diagnostic_tx_gain_is_selectable_but_bounded(self):
        args = self.valid_args()
        args.tx_gain_db = 50
        Program.validate_arguments(args)
        args.tx_gain_db = 71
        with self.assertRaises(ValueError):
            Program.validate_arguments(args)

    def test_measured_hardware_delay_can_be_applied_explicitly(self):
        args = self.valid_args()
        args.rx_start_us = 0
        args.expected_hardware_delay_samples = 166
        Program.validate_arguments(args)
        scheduled = round(
            (args.tx_test_offset_us-args.rx_start_us)*40
        )
        self.assertEqual(
            scheduled+args.expected_hardware_delay_samples,
            566,
        )

    def test_each_tx_is_a_finite_timed_burst(self):
        with patch.object(Program, "uhd", FakeUhd):
            metadata = Program.tx_metadata(12.5)
        self.assertTrue(metadata.has_time_spec)
        self.assertTrue(metadata.start_of_burst)
        self.assertTrue(metadata.end_of_burst)
        self.assertEqual(metadata.time_spec.value, 12.5)

    def test_matched_filter_recovers_known_lag(self):
        rng = np.random.default_rng(4)
        waveform = np.exp(1j*np.arange(200)*0.13).astype(np.complex64)
        iq = (rng.normal(size=1000)+1j*rng.normal(size=1000)).astype(np.complex64)*0.01
        iq[160:360] += waveform
        lag, peak_db = Program.matched_filter_metrics(iq, waveform, 160)
        self.assertEqual(lag, 160)
        self.assertGreater(peak_db, 20.0)

    def test_nonblocking_async_drain_counts_acks_and_reports_errors(self):
        metadata = type("Metadata", (), {})()
        errors = []
        count = Program.drain_tx_events_nonblocking(
            FakeAsyncStreamer([0x01, 0x02, 0x01]), metadata, 0, errors
        )
        self.assertEqual(count, 2)
        self.assertEqual(errors, ["underflow"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
