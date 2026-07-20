"""Hardware-free safety tests for the Stage 3G ATR CRO harness."""

from pathlib import Path
from types import SimpleNamespace
import unittest

import RunEttusAtrCroVerification as cro


class FakeUsrp:
    def __init__(self):
        self.registers = {
            "CTRL": 0,
            "DDR": 0,
            "OUT": 0,
            "ATR_0X": 0,
            "ATR_RX": 0,
            "ATR_TX": 0,
            "ATR_XX": 0,
        }
        self.writes = []

    def get_gpio_banks(self, _mboard):
        return ["FP0"]

    def set_gpio_attr(self, bank, attribute, value, mask, mboard):
        self.assert_call(bank, mask, mboard)
        current = self.registers.get(attribute, 0)
        self.registers[attribute] = (current & ~mask) | (value & mask)
        self.writes.append((attribute, value & mask))

    def get_gpio_attr(self, bank, attribute, mboard):
        if bank != "FP0" or mboard != 0:
            raise AssertionError("wrong GPIO bank or motherboard")
        return self.registers[attribute]

    @staticmethod
    def assert_call(bank, mask, mboard):
        if bank != "FP0" or mask != cro.ATR_MASK or mboard != 0:
            raise AssertionError("wrong GPIO write target")


def safe_args(**overrides):
    values = {
        "prf_hz": 2000.0,
        "pulses": 32,
        "queue_depth": 20,
        "lead_ms": 5.0,
        "dwells": 10,
        "continuous": False,
        "timeout_sec": 1.0,
        "idle_observation_sec": 2.0,
        "shutdown_observation_sec": 10.0,
        "i_confirm_trm_and_pa_disconnected": True,
        "i_confirm_txrx_terminated_50_ohm": True,
        "i_confirm_cro_inputs_high_impedance": True,
        "i_understand_zero_iq_still_enables_tx_chain": True,
    }
    values.update(overrides)
    return SimpleNamespace(**values)


class TestEttusAtrCroSafety(unittest.TestCase):
    def test_confirmed_physical_mapping(self):
        self.assertEqual(cro.GPIO_BANK, "FP0")
        self.assertEqual(cro.TX_ATR_BIT, 1)
        self.assertEqual(cro.RX_ATR_BIT, 2)
        self.assertEqual(cro.OVERLAP_BIT, 3)

    def test_atr_state_words_fail_operational_outputs_low_on_overlap(self):
        fake = FakeUsrp()
        controller = cro.AtrCroController(fake)
        controller.configure_atr()

        self.assertEqual(fake.registers["ATR_0X"] & cro.ATR_MASK, 0)
        self.assertEqual(
            fake.registers["ATR_RX"] & cro.ATR_MASK,
            cro.RX_ATR_MASK,
        )
        self.assertEqual(
            fake.registers["ATR_TX"] & cro.ATR_MASK,
            cro.TX_ATR_MASK,
        )
        self.assertEqual(
            fake.registers["ATR_XX"] & cro.ATR_MASK,
            cro.OVERLAP_MASK,
        )
        self.assertEqual(
            fake.registers["ATR_XX"]
            & (cro.TX_ATR_MASK | cro.RX_ATR_MASK),
            0,
        )

    def test_atr_control_is_enabled_only_after_state_words(self):
        fake = FakeUsrp()
        controller = cro.AtrCroController(fake)
        controller.configure_atr()

        self.assertEqual(fake.writes[-1], ("CTRL", cro.ATR_MASK))

    def test_shutdown_forces_manual_low(self):
        fake = FakeUsrp()
        controller = cro.AtrCroController(fake)
        controller.configure_atr()
        controller.shutdown_safe_low()

        self.assertEqual(fake.registers["CTRL"] & cro.ATR_MASK, 0)
        self.assertEqual(fake.registers["DDR"] & cro.ATR_MASK, cro.ATR_MASK)
        self.assertEqual(fake.registers["OUT"] & cro.ATR_MASK, 0)

    def test_every_physical_acknowledgement_is_required(self):
        confirmation_names = (
            "i_confirm_trm_and_pa_disconnected",
            "i_confirm_txrx_terminated_50_ohm",
            "i_confirm_cro_inputs_high_impedance",
            "i_understand_zero_iq_still_enables_tx_chain",
        )
        for name in confirmation_names:
            with self.subTest(name=name):
                with self.assertRaisesRegex(ValueError, "Missing"):
                    cro.validate_arguments(safe_args(**{name: False}))

    def test_proven_timing_baseline_is_retained(self):
        cro.validate_arguments(safe_args())
        self.assertEqual(cro.SAMPLE_RATE_HZ, 40.0e6)
        self.assertEqual(cro.TX_NUM_SAMPLES, 200)
        self.assertAlmostEqual(cro.TX_DURATION_SEC, 5.0e-6)
        self.assertAlmostEqual(cro.RX_START_DELAY_SEC, 6.0e-6)
        self.assertEqual(cro.RX_NUM_SAMPLES, 4043)
        self.assertAlmostEqual(cro.DWELL_CADENCE_SEC, 0.100)

    def test_five_ms_lead_and_depth_twenty_are_maximum_test_boundary(self):
        cro.validate_arguments(safe_args(lead_ms=5.0, queue_depth=20))
        with self.assertRaisesRegex(ValueError, "at least 5 ms"):
            cro.validate_arguments(safe_args(lead_ms=4.999))
        with self.assertRaisesRegex(ValueError, "between 1 and 20"):
            cro.validate_arguments(safe_args(queue_depth=21))

    def test_continuous_mode_does_not_require_finite_dwell_count(self):
        cro.validate_arguments(safe_args(continuous=True, dwells=0))

    def test_shutdown_observation_defaults_to_zero(self):
        args = cro.parse_arguments([])
        self.assertEqual(args.shutdown_observation_sec, 0.0)

    def test_operational_main_is_not_modified_to_enable_atr(self):
        candidates = (
            Path(__file__).with_name("VanguardxMain_scheduler.py"),
            Path(__file__).parent / "upload" / "VanguardxMain_scheduler(8).py",
        )
        main_path = next(path for path in candidates if path.exists())
        main_text = main_path.read_text(encoding="utf-8")
        self.assertIn('"EttusOperatingMode": "RECEIVE_ONLY"', main_text)
        self.assertIn('"EttusTimedTransmitEnabled": False', main_text)
        self.assertIn('"EttusAtrGpioEnabled": False', main_text)

    def test_stage3e1_source_guard_remains_present(self):
        candidates = (
            Path(__file__).with_name("EttusRadarSource.py"),
            Path(__file__).parent / "upload" / "EttusRadarSource(6).py",
        )
        source_path = next(path for path in candidates if path.exists())
        source_text = source_path.read_text(encoding="utf-8")
        self.assertIn("EttusAtrCroVerifiedAcknowledged", source_text)


if __name__ == "__main__":
    unittest.main()
