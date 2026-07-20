"""Hardware-free checks for noise-referenced range-profile scaling."""

import unittest

from RangeProfileScaling import CalculateNoiseReferencedRangeProfileLimits


class TestRangeProfileScaling(unittest.TestCase):
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
