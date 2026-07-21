import struct
import unittest

import RunX660DirectionCalibration as Harness
from X660CalibrationProtocol import (
    ABSOLUTE_POSITION_COMMAND,
    MOTOR_STOP_COMMAND,
    BuildAbsolutePositionRequest,
    BuildMotorStopRequest,
    ValidateMotionReply,
)


class TestX660DirectionCalibrationSafety(unittest.TestCase):
    def test_defaults_to_dry_run(self):
        Arguments = Harness.ParseArguments([])
        Harness.ValidateArguments(Arguments)
        self.assertFalse(Arguments.execute_one_degree_calibration)

    def test_execution_requires_all_three_acknowledgements(self):
        with self.assertRaises(ValueError):
            Harness.ValidateArguments(
                Harness.ParseArguments(["--execute-one-degree-calibration"])
            )

    def test_fixed_calibration_limits_are_conservative(self):
        self.assertEqual(Harness.CALIBRATION_DELTA_DEG, 1.0)
        self.assertEqual(Harness.CALIBRATION_SPEED_DEG_PER_SEC, 1)
        self.assertLessEqual(Harness.MAX_TRAVEL_FROM_START_DEG, 1.5)
        self.assertLessEqual(Harness.MAX_ABS_CURRENT_A, 2.0)

    def test_absolute_position_frame_is_exact(self):
        Frame = BuildAbsolutePositionRequest(-360.53, 1)
        self.assertEqual(len(Frame), 8)
        self.assertEqual(Frame[0], ABSOLUTE_POSITION_COMMAND)
        self.assertEqual(Frame[1], 0)
        self.assertEqual(struct.unpack_from("<H", Frame, 2)[0], 1)
        self.assertEqual(struct.unpack_from("<i", Frame, 4)[0], -36053)

    def test_calibration_protocol_rejects_excess_speed(self):
        with self.assertRaises(ValueError):
            BuildAbsolutePositionRequest(0.0, 3)

    def test_stop_frame_is_exact(self):
        self.assertEqual(
            BuildMotorStopRequest(),
            bytes([MOTOR_STOP_COMMAND, 0, 0, 0, 0, 0, 0, 0]),
        )

    def test_motion_reply_allow_list_is_narrow(self):
        ValidateMotionReply(ABSOLUTE_POSITION_COMMAND, bytes([0xA4]) + bytes(7))
        ValidateMotionReply(MOTOR_STOP_COMMAND, bytes([0x81]) + bytes(7))
        with self.assertRaises(ValueError):
            ValidateMotionReply(0xA2, bytes([0xA2]) + bytes(7))


if __name__ == "__main__":
    unittest.main()
