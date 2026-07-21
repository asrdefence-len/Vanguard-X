import struct
import unittest

import RunX660VisibleDirectionCalibration as Harness
from X660CalibrationProtocol import ABSOLUTE_POSITION_COMMAND


class TestX660VisibleDirectionCalibrationSafety(unittest.TestCase):
    def test_defaults_to_dry_run(self):
        Arguments = Harness.ParseArguments([])
        Harness.ValidateArguments(Arguments)
        self.assertFalse(Arguments.execute_five_degree_calibration)

    def test_execution_requires_all_three_acknowledgements(self):
        with self.assertRaises(ValueError):
            Harness.ValidateArguments(
                Harness.ParseArguments(["--execute-five-degree-calibration"])
            )

    def test_fixed_visible_calibration_envelope(self):
        self.assertEqual(Harness.CALIBRATION_DELTA_DEG, 5.0)
        self.assertEqual(Harness.CALIBRATION_SPEED_DEG_PER_SEC, 1)
        self.assertEqual(Harness.MAX_TRAVEL_FROM_START_DEG, 5.5)
        self.assertLessEqual(Harness.MAX_ABS_CURRENT_A, 2.0)
        self.assertLessEqual(Harness.MAX_REPORTED_SPEED_DEG_PER_SEC, 3)
        self.assertGreaterEqual(Harness.MOVE_TIMEOUT_SEC, 10.0)

    def test_target_is_exactly_five_raw_degrees_positive(self):
        self.assertAlmostEqual(Harness.CalculateTargetRawAngle(-360.53), -355.53)

    def test_absolute_position_frame_is_exact(self):
        Frame = Harness.BuildCalibrationFrame(-360.53)
        self.assertEqual(len(Frame), 8)
        self.assertEqual(Frame[0], ABSOLUTE_POSITION_COMMAND)
        self.assertEqual(Frame[1], 0)
        self.assertEqual(struct.unpack_from("<H", Frame, 2)[0], 1)
        self.assertEqual(struct.unpack_from("<i", Frame, 4)[0], -35553)

    def test_all_acknowledgements_authorise_validation_only(self):
        Arguments = Harness.ParseArguments(
            [
                "--execute-five-degree-calibration",
                "--i-confirm-x6-60-is-secured-and-area-clear",
                "--i-confirm-at-least-six-degrees-free-rotation",
                "--i-confirm-power-isolation-is-immediately-accessible",
            ]
        )
        Harness.ValidateArguments(Arguments)
        self.assertTrue(Arguments.execute_five_degree_calibration)


if __name__ == "__main__":
    unittest.main()
