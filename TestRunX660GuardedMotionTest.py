#!/usr/bin/env python3
"""Fake-bus safety tests for the Stage 5C standalone X6-60 harness."""

from __future__ import annotations

import ast
from pathlib import Path
from types import SimpleNamespace
import unittest

import RunX660GuardedMotionTest as Harness


class FakeMessage:
    def __init__(self, arbitration_id, data, is_extended_id=False):
        self.arbitration_id = arbitration_id
        self.data = bytes(data)
        self.is_extended_id = is_extended_id


class FakeCanModule:
    Message = FakeMessage


class ReplyingFakeBus:
    def __init__(self):
        self.Sent = []
        self.Replies = []
        self.ShutdownCalled = False

    def send(self, Message, timeout=None):
        self.Sent.append((Message, timeout))
        self.Replies.append(
            FakeMessage(
                arbitration_id=0x241,
                data=bytes(Message.data),
                is_extended_id=False,
            )
        )

    def recv(self, timeout=None):
        if self.Replies:
            return self.Replies.pop(0)
        return None

    def shutdown(self):
        self.ShutdownCalled = True


def Arguments(**Overrides):
    Values = dict(
        Interface="can0",
        NodeId=1,
        IncrementDeg=1.0,
        MaxSpeedDegPerSec=2,
        TimeoutSec=0.02,
        LiveMotion=False,
        IUnderstandMotionWillOccur=False,
        IConfirmMotionAreaIsClear=False,
        IConfirmAxisIsSupported=False,
        IConfirmEmergencyStopIsReady=False,
    )
    Values.update(Overrides)
    return SimpleNamespace(**Values)


def ArmedArguments(**Overrides):
    Values = dict(
        LiveMotion=True,
        IUnderstandMotionWillOccur=True,
        IConfirmMotionAreaIsClear=True,
        IConfirmAxisIsSupported=True,
        IConfirmEmergencyStopIsReady=True,
    )
    Values.update(Overrides)
    return Arguments(**Values)


class TestRunX660GuardedMotionTest(unittest.TestCase):
    def test_dry_run_is_default_and_exact_plan_is_bounded(self):
        Args = Harness.BuildArgumentParser().parse_args([])
        self.assertFalse(Args.LiveMotion)
        Plan = Harness.BuildDryRunPlan(Args)
        self.assertEqual(Plan.MotionRequest.arbitration_id, 0x141)
        self.assertEqual(
            bytes(Plan.MotionRequest.data),
            bytes.fromhex("A8 00 02 00 64 00 00 00"),
        )
        self.assertEqual(
            bytes(Plan.StopRequest.data),
            bytes.fromhex("81 00 00 00 00 00 00 00"),
        )
        self.assertEqual(
            bytes(Plan.ShutdownRequest.data),
            bytes.fromhex("80 00 00 00 00 00 00 00"),
        )
        self.assertEqual(Plan.EstimatedMotionTimeSec, 1.0)

    def test_motion_envelope_rejects_unbounded_or_imprecise_values(self):
        Invalid = (
            (0.0, 2),
            (2.01, 2),
            (-2.01, 2),
            (0.001, 2),
            (float("nan"), 2),
            (1.0, 0),
            (1.0, 6),
            (1.0, 1.5),
            (True, 2),
            (1.0, True),
        )
        for Increment, Speed in Invalid:
            with self.subTest(Increment=Increment, Speed=Speed):
                with self.assertRaises(ValueError):
                    Harness.ValidateMotionLimits(Increment, Speed)

    def test_live_mode_requires_every_explicit_acknowledgement(self):
        Flags = (
            "IUnderstandMotionWillOccur",
            "IConfirmMotionAreaIsClear",
            "IConfirmAxisIsSupported",
            "IConfirmEmergencyStopIsReady",
        )
        for Missing in Flags:
            Overrides = {Name: True for Name in Flags}
            Overrides[Missing] = False
            with self.subTest(Missing=Missing):
                with self.assertRaisesRegex(RuntimeError, "requires all"):
                    Harness.ExecuteLive(
                        Arguments(LiveMotion=True, **Overrides),
                        CanModule=FakeCanModule,
                        BusFactory=lambda **Unused: ReplyingFakeBus(),
                        InterfaceChecker=lambda Interface: None,
                        Sleep=lambda Seconds: None,
                    )

    def test_execute_live_refuses_without_live_mode(self):
        with self.assertRaisesRegex(RuntimeError, "explicit live-motion"):
            Harness.ExecuteLive(Arguments())

    def test_fake_live_sequence_is_increment_stop_then_shutdown(self):
        Bus = ReplyingFakeBus()
        Checked = []
        Slept = []
        Result = Harness.ExecuteLive(
            ArmedArguments(),
            CanModule=FakeCanModule,
            BusFactory=lambda **Fields: Bus,
            InterfaceChecker=Checked.append,
            Sleep=Slept.append,
        )
        self.assertEqual(Checked, ["can0"])
        self.assertEqual(Slept, [1.0])
        self.assertEqual(
            [bytes(Message.data)[0] for Message, Timeout in Bus.Sent],
            [0xA8, 0x81, 0x80],
        )
        self.assertTrue(Bus.ShutdownCalled)
        self.assertEqual(Result.ShutdownReply, bytes([0x80]) + bytes(7))

    def test_motion_failure_still_attempts_stop_and_shutdown(self):
        class MotionReplyTimeoutBus(ReplyingFakeBus):
            def send(self, Message, timeout=None):
                self.Sent.append((Message, timeout))
                if bytes(Message.data)[0] != 0xA8:
                    self.Replies.append(
                        FakeMessage(0x241, bytes(Message.data), False)
                    )

        Bus = MotionReplyTimeoutBus()
        with self.assertRaisesRegex(RuntimeError, "shutdown was confirmed"):
            Harness.ExecuteLive(
                ArmedArguments(TimeoutSec=0.01),
                CanModule=FakeCanModule,
                BusFactory=lambda **Fields: Bus,
                InterfaceChecker=lambda Interface: None,
                Sleep=lambda Seconds: None,
            )
        self.assertEqual(
            [bytes(Message.data)[0] for Message, Timeout in Bus.Sent],
            [0xA8, 0x81, 0x80],
        )
        self.assertTrue(Bus.ShutdownCalled)

    def test_shutdown_failure_is_reported_as_fail_safe_error(self):
        class ShutdownFailureBus(ReplyingFakeBus):
            def send(self, Message, timeout=None):
                self.Sent.append((Message, timeout))
                if bytes(Message.data)[0] != 0x80:
                    self.Replies.append(
                        FakeMessage(0x241, bytes(Message.data), False)
                    )

        Bus = ShutdownFailureBus()
        with self.assertRaisesRegex(RuntimeError, "FAIL-SAFE ERROR"):
            Harness.ExecuteLive(
                ArmedArguments(TimeoutSec=0.01),
                CanModule=FakeCanModule,
                BusFactory=lambda **Fields: Bus,
                InterfaceChecker=lambda Interface: None,
                Sleep=lambda Seconds: None,
            )
        self.assertTrue(Bus.ShutdownCalled)

    def test_harness_is_standalone_and_does_not_import_operational_paths(self):
        Source = Path(Harness.__file__).read_text(encoding="utf-8")
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
                {
                    "X660Controller",
                    "X660ReadOnlyController",
                    "PointingManager",
                    "RadarScheduler",
                    "VanguardxMain_scheduler",
                }
            )
        )
        self.assertNotIn("BuildSpeedControlRequest", Source)
        self.assertNotIn("BuildAbsolutePositionRequest", Source)


if __name__ == "__main__":
    unittest.main(verbosity=2)
