"""Regression tests for encoder-based X6-60 startup initialisation."""

import ast
import types
import unittest
from pathlib import Path


SCHEDULER_PATH = Path(__file__).with_name("VanguardxMain_scheduler.py")


def _load_startup_helper():
    source = SCHEDULER_PATH.read_text(encoding="utf-8")
    module = ast.parse(source, filename=str(SCHEDULER_PATH))
    helper = next(
        node
        for node in module.body
        if isinstance(node, ast.FunctionDef)
        and node.name == "InitialiseX660AtCurrentPose"
    )
    isolated = ast.Module(body=[helper], type_ignores=[])
    namespace = {}
    exec(compile(isolated, str(SCHEDULER_PATH), "exec"), namespace)
    return namespace["InitialiseX660AtCurrentPose"]


InitialiseX660AtCurrentPose = _load_startup_helper()


class _Display:
    def __init__(self):
        self.measured_angles = []

    def SetMeasuredBeamAngle(self, angle_deg):
        self.measured_angles.append(float(angle_deg))


class _Controller:
    def __init__(self, valid=True):
        self.state = types.SimpleNamespace(
            Valid=bool(valid),
            AzimuthDeg=372.5,
            ElevationDeg=0.0,
            RawAngleDeg=-349.03,
        )
        self.update_count = 0
        self.motion_commands = []

    def Update(self):
        self.update_count += 1
        return self.state

    def CommandPosition(self, *args):
        self.motion_commands.append(("CommandPosition", args))

    def SetPanPositionNative(self, *args):
        self.motion_commands.append(("SetPanPositionNative", args))

    def CommandSlew(self, *args):
        self.motion_commands.append(("CommandSlew", args))

    def Stop(self, *args):
        self.motion_commands.append(("Stop", args))


class X660StartupCurrentPositionTests(unittest.TestCase):
    def test_adopts_encoder_pose_without_any_motion_command(self):
        controller = _Controller()
        display = _Display()
        config = {
            "InitialBeamAngleDeg": 200.0,
            "BoresightDeg": 200.0,
        }

        returned_state = InitialiseX660AtCurrentPose(
            controller,
            config,
            display,
        )

        self.assertIs(returned_state, controller.state)
        self.assertEqual(controller.update_count, 1)
        self.assertEqual(controller.motion_commands, [])
        self.assertAlmostEqual(config["InitialBeamAngleDeg"], 12.5)
        self.assertAlmostEqual(config["BoresightDeg"], 12.5)
        self.assertAlmostEqual(config["AntennaAzDeg"], 12.5)
        self.assertAlmostEqual(config["X660AzDeg"], 12.5)
        self.assertAlmostEqual(config["X660RawAngleDeg"], -349.03)
        self.assertEqual(display.measured_angles, [12.5])

    def test_invalid_encoder_state_fails_without_motion(self):
        controller = _Controller(valid=False)

        with self.assertRaisesRegex(RuntimeError, "telemetry is invalid"):
            InitialiseX660AtCurrentPose(controller, {}, _Display())

        self.assertEqual(controller.motion_commands, [])

    def test_none_controller_is_a_no_op(self):
        config = {"InitialBeamAngleDeg": 123.0}

        result = InitialiseX660AtCurrentPose(None, config, _Display())

        self.assertIsNone(result)
        self.assertEqual(config["InitialBeamAngleDeg"], 123.0)


if __name__ == "__main__":
    unittest.main()
