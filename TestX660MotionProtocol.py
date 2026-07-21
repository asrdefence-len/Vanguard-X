#!/usr/bin/env python3
"""Software-only Stage 5A tests for X6-60 motion payload encoding."""

from __future__ import annotations

import ast
from pathlib import Path
import unittest

import X660MotionProtocol as Protocol
from X660Controller import CreateX660Controller
from X660ReadOnlyController import X660ReadOnlyController


class TestX660MotionProtocol(unittest.TestCase):
    def test_manual_stop_vector(self):
        self.assertEqual(
            Protocol.BuildMotorStopRequest(),
            bytes.fromhex("81 00 00 00 00 00 00 00"),
        )

    def test_manual_speed_vectors(self):
        self.assertEqual(
            Protocol.BuildSpeedControlRequest(100.0),
            bytes.fromhex("A2 00 00 00 10 27 00 00"),
        )
        self.assertEqual(
            Protocol.BuildSpeedControlRequest(-100.0),
            bytes.fromhex("A2 00 00 00 F0 D8 FF FF"),
        )

    def test_manual_absolute_position_vectors(self):
        self.assertEqual(
            Protocol.BuildAbsolutePositionRequest(360.0, 500),
            bytes.fromhex("A4 00 F4 01 A0 8C 00 00"),
        )
        self.assertEqual(
            Protocol.BuildAbsolutePositionRequest(-360.0, 500),
            bytes.fromhex("A4 00 F4 01 60 73 FF FF"),
        )

    def test_manual_incremental_position_vectors(self):
        self.assertEqual(
            Protocol.BuildIncrementalPositionRequest(360.0, 500),
            bytes.fromhex("A8 00 F4 01 A0 8C 00 00"),
        )
        self.assertEqual(
            Protocol.BuildIncrementalPositionRequest(-360.0, 500),
            bytes.fromhex("A8 00 F4 01 60 73 FF FF"),
        )

    def test_vanguard_operational_rate_and_calibrated_raw_angle(self):
        self.assertEqual(
            Protocol.BuildSpeedControlRequest(14.0),
            bytes.fromhex("A2 00 00 00 78 05 00 00"),
        )
        self.assertEqual(
            Protocol.BuildAbsolutePositionRequest(-361.53, 2),
            bytes.fromhex("A4 00 02 00 C7 72 FF FF"),
        )

    def test_position_speed_limit_cannot_be_zero(self):
        with self.assertRaises(ValueError):
            Protocol.BuildAbsolutePositionRequest(1.0, 0)
        with self.assertRaises(ValueError):
            Protocol.BuildIncrementalPositionRequest(1.0, 0)

    def test_fractional_resolution_is_not_silently_rounded(self):
        with self.assertRaises(ValueError):
            Protocol.BuildSpeedControlRequest(1.001)
        with self.assertRaises(ValueError):
            Protocol.BuildAbsolutePositionRequest(1.001, 2)
        with self.assertRaises(ValueError):
            Protocol.BuildIncrementalPositionRequest(1.001, 2)

    def test_nonfinite_and_out_of_range_values_are_rejected(self):
        for Value in (float("nan"), float("inf"), float("-inf")):
            with self.subTest(Value=Value):
                with self.assertRaises(ValueError):
                    Protocol.BuildSpeedControlRequest(Value)
        with self.assertRaises(ValueError):
            Protocol.BuildSpeedControlRequest((2**31) / 100.0)
        with self.assertRaises(ValueError):
            Protocol.BuildAbsolutePositionRequest((2**31) / 100.0, 2)
        with self.assertRaises(ValueError):
            Protocol.BuildIncrementalPositionRequest(-(2**31 + 1) / 100.0, 2)
        for SpeedLimit in (-1, 65536, 1.5, True):
            with self.subTest(SpeedLimit=SpeedLimit):
                with self.assertRaises(ValueError):
                    Protocol.BuildAbsolutePositionRequest(1.0, SpeedLimit)

    def test_reply_allow_list_command_and_dlc_are_enforced(self):
        for Command in Protocol.MOTION_COMMANDS:
            Payload = bytes([Command]) + bytes(7)
            self.assertEqual(
                Protocol.ValidateMotionReply(Command, Payload),
                Payload,
            )
        with self.assertRaises(ValueError):
            Protocol.ValidateMotionReply(0x77, bytes.fromhex(
                "77 00 00 00 00 00 00 00"
            ))
        with self.assertRaises(ValueError):
            Protocol.ValidateMotionReply(
                Protocol.SPEED_CONTROL_COMMAND,
                bytes.fromhex("A4 00 00 00 00 00 00 00"),
            )
        with self.assertRaises(ValueError):
            Protocol.ValidateMotionReply(
                Protocol.SPEED_CONTROL_COMMAND,
                bytes.fromhex("A2 00 00 00 00 00 00"),
            )

    def test_protocol_module_has_no_hardware_or_transmit_path(self):
        Source = Path(Protocol.__file__).read_text(encoding="utf-8")
        Tree = ast.parse(Source)
        ImportedRoots = set()
        for Node in ast.walk(Tree):
            if isinstance(Node, ast.Import):
                ImportedRoots.update(
                    Alias.name.split(".")[0] for Alias in Node.names
                )
            elif isinstance(Node, ast.ImportFrom) and Node.module:
                ImportedRoots.add(Node.module.split(".")[0])
        self.assertTrue(
            ImportedRoots.isdisjoint(
                {"can", "socket", "subprocess", "serial", "X660CanProtocol"}
            )
        )
        FunctionNames = {
            Node.name for Node in ast.walk(Tree)
            if isinstance(Node, (ast.FunctionDef, ast.AsyncFunctionDef))
        }
        self.assertTrue(
            FunctionNames.isdisjoint(
                {"Open", "Send", "Transmit", "Write", "ReleaseBrake"}
            )
        )

    def test_real_controller_and_factory_remain_motion_locked(self):
        self.assertFalse(X660ReadOnlyController.MotionCommandsEnabled)
        with self.assertRaisesRegex(ValueError, "Unknown X660Mode"):
            CreateX660Controller({"X660Mode": "x660-motion"})


if __name__ == "__main__":
    unittest.main(verbosity=2)
