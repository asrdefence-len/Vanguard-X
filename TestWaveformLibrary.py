"""Regression tests for fixed-rate sampled Vanguard X waveforms."""

import unittest

import numpy as np

from WaveformLibrary import WaveformLibrary


class TestWaveformLibrary(unittest.TestCase):
    def setUp(self):
        self.Library = WaveformLibrary(
            {"EttusSampleRateHz": 40.0e6}
        )
        self.Library.LoadDefaultWaveforms()

    def test_default_catalogue_has_explicit_rate_identifiers(self):
        self.assertEqual(
            self.Library.ListWaveforms(),
            [
                "Barker13_10MHz",
                "Barker13_20MHz",
                "Frank10_10MHz",
                "Frank10_20MHz",
                "Golay64A_20MHz",
                "Golay64B_20MHz",
            ],
        )

        for AmbiguousName in ("Barker13", "Frank10"):
            with self.assertRaisesRegex(ValueError, "Waveform not found"):
                self.Library.Get(AmbiguousName)

    def test_sample_counts_and_pulse_durations(self):
        Expected = {
            "Barker13_10MHz": (13, 10.0e6, 4, 52, 1.30e-6),
            "Barker13_20MHz": (13, 20.0e6, 2, 26, 0.65e-6),
            "Frank10_10MHz": (100, 10.0e6, 4, 400, 10.0e-6),
            "Frank10_20MHz": (100, 20.0e6, 2, 200, 5.0e-6),
            "Golay64A_20MHz": (64, 20.0e6, 2, 128, 3.2e-6),
            "Golay64B_20MHz": (64, 20.0e6, 2, 128, 3.2e-6),
        }

        for WaveformId, Values in Expected.items():
            with self.subTest(WaveformId=WaveformId):
                (
                    ChipCount,
                    ChipRateHz,
                    SamplesPerChip,
                    NumSamples,
                    PulseDurationSec,
                ) = Values
                Metadata = self.Library.GetMetadata(WaveformId)

                self.assertEqual(Metadata["ChipCount"], ChipCount)
                self.assertEqual(Metadata["ChipRateHz"], ChipRateHz)
                self.assertEqual(
                    Metadata["SampleRateHz"],
                    40.0e6,
                )
                self.assertEqual(
                    Metadata["SamplesPerChip"],
                    SamplesPerChip,
                )
                self.assertEqual(Metadata["NumSamples"], NumSamples)
                self.assertAlmostEqual(
                    Metadata["PulseDurationSec"],
                    PulseDurationSec,
                    places=15,
                )
                self.assertEqual(
                    NumSamples / Metadata["SampleRateHz"],
                    PulseDurationSec,
                )

    def test_sampled_waveform_repeats_each_chip(self):
        for WaveformId in self.Library.ListWaveforms():
            with self.subTest(WaveformId=WaveformId):
                Definition = self.Library.GetDefinition(WaveformId)
                Reshaped = Definition.Samples.reshape(
                    Definition.ChipCount,
                    Definition.SamplesPerChip,
                )
                Expected = np.repeat(
                    Definition.Chips[:, np.newaxis],
                    Definition.SamplesPerChip,
                    axis=1,
                )
                np.testing.assert_array_equal(Reshaped, Expected)
                np.testing.assert_allclose(
                    np.abs(Definition.Chips),
                    1.0,
                    rtol=0.0,
                    atol=1e-6,
                )
                np.testing.assert_allclose(
                    np.abs(Definition.Samples),
                    1.0,
                    rtol=0.0,
                    atol=1e-6,
                )

    def test_definition_arrays_are_read_only(self):
        Definition = self.Library.GetDefinition("Frank10_20MHz")
        self.assertFalse(Definition.Chips.flags.writeable)
        self.assertFalse(Definition.Samples.flags.writeable)

        with self.assertRaises(ValueError):
            Definition.Samples[0] = 0.0

    def test_processing_gain_uses_chip_count(self):
        self.assertAlmostEqual(
            self.Library.GetIdealProcessingGainDb("Barker13_20MHz"),
            10.0 * np.log10(13),
            places=12,
        )
        self.assertAlmostEqual(
            self.Library.GetIdealProcessingGainDb("Frank10_20MHz"),
            20.0,
            places=12,
        )
        self.assertEqual(
            self.Library.GetCodeLength("Frank10_20MHz"),
            100,
        )
        self.assertEqual(
            self.Library.GetSampleCount("Frank10_20MHz"),
            200,
        )

    def test_non_integer_samples_per_chip_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "integer number of samples"):
            self.Library.RegisterPhaseCode(
                WaveformId="Test4_12MHz",
                CodeFamily="TEST",
                Chips=np.ones(4, dtype=np.complex64),
                ChipRateHz=12.0e6,
            )

    def test_rate_free_identifier_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "must end with '_20MHz'"):
            self.Library.RegisterPhaseCode(
                WaveformId="Test4",
                CodeFamily="TEST",
                Chips=np.ones(4, dtype=np.complex64),
                ChipRateHz=20.0e6,
            )

    def test_non_unit_phase_code_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "unit magnitude"):
            self.Library.RegisterPhaseCode(
                WaveformId="Test4_20MHz",
                CodeFamily="TEST",
                Chips=np.array([1.0, 1.0, 0.5, -1.0]),
                ChipRateHz=20.0e6,
            )

    def test_non_40_msps_configuration_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "fixed 40 MS/s"):
            WaveformLibrary({"EttusSampleRateHz": 20.0e6})


if __name__ == "__main__":
    unittest.main(verbosity=2)
