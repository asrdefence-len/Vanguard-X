"""Hardware-free checks for noise-referenced range-profile scaling."""

import unittest

import numpy as np

from RangeProfileScaling import (
    CalculateNoiseReferencedRangeProfileLimits,
    SelectRangeProfileDb,
)


class TestRangeProfileScaling(unittest.TestCase):
    def test_zero_doppler_mode_selects_the_bin_nearest_zero_hz(self):
        magnitude_db = np.array([
            [10.0, 11.0, 12.0],
            [20.0, 21.0, 22.0],
            [30.0, 31.0, 32.0],
        ])
        doppler_hz = np.array([-1000.0, 0.25, 1000.0])

        result = SelectRangeProfileDb(
            magnitude_db,
            doppler_hz,
            Mode="ZERO_DOPPLER",
        )

        np.testing.assert_array_equal(result, magnitude_db[1])

    def test_max_mode_retains_the_operational_envelope(self):
        magnitude_db = np.array([
            [1.0, 9.0],
            [8.0, 2.0],
        ])
        np.testing.assert_array_equal(
            SelectRangeProfileDb(magnitude_db, Mode="MAX"),
            np.array([8.0, 9.0]),
        )

    def test_zero_doppler_mode_requires_a_matching_axis(self):
        with self.assertRaises(ValueError):
            SelectRangeProfileDb(
                np.zeros((3, 4)),
                np.zeros(2),
                Mode="ZERO_DOPPLER",
            )

    def test_axis_follows_high_absolute_noise_and_peak(self):
        minimum_db, maximum_db = (
            CalculateNoiseReferencedRangeProfileLimits(
                NoiseFloorDb=-12.0,
                PeakDb=34.0,
            )
        )
        self.assertEqual(minimum_db, -17.0)
        self.assertEqual(maximum_db, 37.0)

    def test_noise_only_profile_keeps_minimum_visible_span(self):
        minimum_db, maximum_db = (
            CalculateNoiseReferencedRangeProfileLimits(
                NoiseFloorDb=-45.0,
                PeakDb=-39.0,
            )
        )
        self.assertEqual(minimum_db, -50.0)
        self.assertEqual(maximum_db, -25.0)

    def test_invalid_limits_are_rejected(self):
        with self.assertRaises(ValueError):
            CalculateNoiseReferencedRangeProfileLimits(
                NoiseFloorDb=-45.0,
                PeakDb=-39.0,
                MinimumSpanAboveNoiseDb=0.0,
            )


if __name__ == "__main__":
    unittest.main()
