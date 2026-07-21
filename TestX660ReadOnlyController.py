import unittest

from X660ReadOnlyController import (
    RawAngleToAzimuthDeg,
    X660ReadOnlyController,
)


class TestX660ReadOnlyController(unittest.TestCase):
    def test_raw_angle_mapping_both_possible_directions(self):
        self.assertAlmostEqual(
            RawAngleToAzimuthDeg(-361.53, -361.53, 1),
            0.0,
        )
        self.assertAlmostEqual(
            RawAngleToAzimuthDeg(-308.77, -361.53, 1),
            52.76,
        )
        self.assertAlmostEqual(
            RawAngleToAzimuthDeg(-308.77, -361.53, -1),
            307.24,
        )

    def test_uncalibrated_direction_is_explicit(self):
        Controller = X660ReadOnlyController(DirectionSign=0)
        self.assertFalse(Controller.Calibrated)
        self.assertFalse(Controller.MotionCommandsEnabled)
        self.assertFalse(Controller.State.Valid)

    def test_all_pointing_commands_remain_locked(self):
        Controller = X660ReadOnlyController(DirectionSign=1)
        Controller.CommandSlew(1.0)
        Controller.SetPanPositionNative(20.0)
        Controller.NudgePanPositionNative(1.0)
        Controller.SetTiltPositionNative(10.0)
        Controller.CommandPosition(20.0, 10.0)
        Controller.Stop()
        Controller.StopAndHold()
        self.assertFalse(Controller.State.Valid)
        self.assertEqual(
            Controller.State.Source,
            "X660_CAN_READ_ONLY_MOTION_LOCKED",
        )
        self.assertIsNone(Controller.Bus)

    def test_rejects_invalid_direction_sign(self):
        with self.assertRaisesRegex(ValueError, "DirectionSign"):
            X660ReadOnlyController(DirectionSign=2)


if __name__ == "__main__":
    unittest.main()
