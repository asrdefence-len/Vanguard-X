"""Checks that near-range cells cannot seed the tracker through CFAR."""

from types import SimpleNamespace
import unittest

import numpy as np

from CfarDetector import CfarDetector


class TestMinimumDetectionRange(unittest.TestCase):
    def test_cfar_discards_hits_below_two_kilometres(self):
        detector = CfarDetector({
            "TrainingCellsRange": 1,
            "TrainingCellsDoppler": 1,
            "GuardCellsRange": 0,
            "GuardCellsDoppler": 0,
            "CfarThresholdDb": 6.0,
            "MaxDetections": 20,
            "MinRangeM": 2000.0,
            "MaxRangeM": 15000.0,
        })

        range_axis_m = np.arange(20, dtype=float) * 500.0
        doppler_axis_hz = np.arange(7, dtype=float)
        velocity_axis_mps = doppler_axis_hz.copy()
        range_doppler = np.ones((7, 20), dtype=np.complex64)

        # Strong direct-path/recovery artefact at 1.5 km.
        range_doppler[3, 3] = 100.0 + 0.0j
        # Valid target at 5 km.
        range_doppler[3, 10] = 80.0 + 0.0j

        processed = SimpleNamespace(
            RangeDopplerMap=range_doppler,
            RangeAxisM=range_axis_m,
            DopplerAxisHz=doppler_axis_hz,
            VelocityAxisMps=velocity_axis_mps,
            Diagnostics={"BoresightDeg": 80.0},
            DwellId=1,
            TimeStamp=0.0,
        )

        detections = detector.Detect(processed)

        self.assertGreaterEqual(len(detections), 1)
        self.assertTrue(all(d.RangeM >= 2000.0 for d in detections))
        self.assertTrue(any(d.RangeM == 5000.0 for d in detections))
        self.assertFalse(any(d.RangeM == 1500.0 for d in detections))

    def test_main_configures_two_kilometre_detection_blanking(self):
        with open("VanguardxMain_scheduler.py", "r", encoding="utf-8") as source:
            main_text = source.read()
        self.assertIn('"MinRangeM": 2000.0', main_text)


if __name__ == "__main__":
    unittest.main()
