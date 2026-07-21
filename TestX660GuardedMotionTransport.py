#!/usr/bin/env python3
"""Software-only Stage 5B tests for the guarded X6-60 motion transport."""

from __future__ import annotations

import ast
from pathlib import Path
import unittest

import X660GuardedMotionTransport as Transport
import X660MotionProtocol as MotionProtocol
from X660Controller import CreateX660Controller
from X660ReadOnlyController import X660ReadOnlyController


class FakeBus:
    def __init__(self, Replies=()):
        self.Replies = list(Replies)
        self.Sent = []
        self.ReceiveTimeouts = []

    def send(self, Message, timeout=None):
        self.Sent.append((Message, timeout))

    def recv(self, timeout=None):
        self.ReceiveTimeouts.append(timeout)
        if self.Replies:
            return self.Replies.pop(0)
        return None


def ReplyFrame(Command, NodeId=1, Data=None, Extended=False):
    Payload = Data if Data is not None else bytes([Command]) + bytes(7)
    return Transport.CanFrame(
        arbitration_id=Transport.ReplyArbitrationId(NodeId),
        data=Payload,
        is_extended_id=Extended,
    )


class TestX660GuardedMotionTransport(unittest.TestCase):
    def test_documented_node_one_standard_identifiers(self):
        self.assertEqual(Transport.RequestArbitrationId(1), 0x141)
        self.assertEqual(Transport.ReplyArbitrationId(1), 0x241)
        for NodeId in (0, 33, 1.5, True):
            with self.subTest(NodeId=NodeId):
                with self.assertRaises(ValueError):
                    Transport.RequestArbitrationId(NodeId)

    def test_dry_run_is_default_and_does_not_touch_injected_bus(self):
        Bus = FakeBus()
        Guard = Transport.X660GuardedMotionTransport(Bus=Bus)
        Result = Guard.Transact(
            MotionProtocol.BuildSpeedControlRequest(14.0)
        )
        self.assertTrue(Result.DryRun)
        self.assertFalse(Result.Transmitted)
        self.assertIsNone(Result.ReplyData)
        self.assertEqual(Bus.Sent, [])
        self.assertEqual(Bus.ReceiveTimeouts, [])
        self.assertEqual(Result.Request.arbitration_id, 0x141)
        self.assertFalse(Result.Request.is_extended_id)
        self.assertEqual(
            Result.Request.data,
            bytes.fromhex("A2 00 00 00 78 05 00 00"),
        )

    def test_all_four_stage5a_commands_can_be_previewed(self):
        Payloads = (
            MotionProtocol.BuildMotorStopRequest(),
            MotionProtocol.BuildSpeedControlRequest(-14.0),
            MotionProtocol.BuildAbsolutePositionRequest(-361.53, 2),
            MotionProtocol.BuildIncrementalPositionRequest(1.0, 2),
        )
        Guard = Transport.X660GuardedMotionTransport()
        for Payload in Payloads:
            with self.subTest(Command=Payload[0]):
                Result = Guard.Transact(Payload)
                self.assertEqual(Result.Command, Payload[0])
                self.assertEqual(Result.Request.data, Payload)
                self.assertFalse(Result.Transmitted)

    def test_non_motion_and_malformed_payloads_are_rejected(self):
        DisallowedCommands = (0x76, 0x77, 0x80, 0x88, 0x9A, 0xB5)
        Guard = Transport.X660GuardedMotionTransport()
        for Command in DisallowedCommands:
            with self.subTest(Command=Command):
                with self.assertRaisesRegex(ValueError, "allow-list"):
                    Guard.Transact(bytes([Command]) + bytes(7))
        for Payload in (
            bytes.fromhex("81 00 00 00 00 00 00"),
            bytes.fromhex("81 00 00 00 00 00 00 00 00"),
            bytes.fromhex("81 01 00 00 00 00 00 00"),
            bytes.fromhex("A2 00 01 00 00 00 00 00"),
            bytes.fromhex("A4 01 02 00 64 00 00 00"),
            bytes.fromhex("A8 00 00 00 64 00 00 00"),
        ):
            with self.subTest(Payload=Payload.hex()):
                with self.assertRaises(ValueError):
                    Guard.Transact(Payload)

    def test_live_mode_requires_bus_and_both_acknowledgements(self):
        Cases = (
            {},
            {"Bus": FakeBus()},
            {"Bus": FakeBus(), "IUnderstandMotionWillOccur": True},
            {"Bus": FakeBus(), "IConfirmMotionAreaIsClear": True},
        )
        for Arguments in Cases:
            with self.subTest(Arguments=Arguments):
                with self.assertRaises(RuntimeError):
                    Transport.X660GuardedMotionTransport(
                        DryRun=False,
                        **Arguments,
                    )

    def test_safety_booleans_and_live_bus_interface_are_strict(self):
        for DryRun in (0, 1, "false", None):
            with self.subTest(DryRun=DryRun):
                with self.assertRaises(ValueError):
                    Transport.X660GuardedMotionTransport(DryRun=DryRun)
        with self.assertRaises(RuntimeError):
            Transport.X660GuardedMotionTransport(
                Bus=object(),
                DryRun=False,
                IUnderstandMotionWillOccur=True,
                IConfirmMotionAreaIsClear=True,
            )
        with self.assertRaises(RuntimeError):
            Transport.X660GuardedMotionTransport(
                Bus=FakeBus(),
                DryRun=False,
                IUnderstandMotionWillOccur=1,
                IConfirmMotionAreaIsClear=True,
            )
        with self.assertRaises(RuntimeError):
            Transport.X660GuardedMotionTransport(
                Bus=FakeBus(),
                DryRun=False,
                IUnderstandMotionWillOccur=True,
                IConfirmMotionAreaIsClear="yes",
            )

    def test_live_fake_bus_sends_exact_standard_frame_and_validates_reply(self):
        Payload = MotionProtocol.BuildAbsolutePositionRequest(-361.53, 2)
        Bus = FakeBus((ReplyFrame(MotionProtocol.ABSOLUTE_POSITION_COMMAND),))
        Guard = Transport.X660GuardedMotionTransport(
            Bus=Bus,
            DryRun=False,
            IUnderstandMotionWillOccur=True,
            IConfirmMotionAreaIsClear=True,
            TimeoutSec=0.02,
        )
        Result = Guard.Transact(Payload)
        self.assertTrue(Result.Transmitted)
        self.assertFalse(Result.DryRun)
        self.assertEqual(
            Result.ReplyData,
            bytes.fromhex("A4 00 00 00 00 00 00 00"),
        )
        self.assertEqual(len(Bus.Sent), 1)
        Request, SendTimeout = Bus.Sent[0]
        self.assertEqual(Request.arbitration_id, 0x141)
        self.assertEqual(Request.data, Payload)
        self.assertFalse(Request.is_extended_id)
        self.assertEqual(SendTimeout, 0.02)

    def test_unrelated_frames_are_ignored_before_valid_reply(self):
        Command = MotionProtocol.SPEED_CONTROL_COMMAND
        Bus = FakeBus((
            Transport.CanFrame(0x242, bytes([Command]) + bytes(7)),
            ReplyFrame(Command, Extended=True),
            ReplyFrame(MotionProtocol.ABSOLUTE_POSITION_COMMAND),
            ReplyFrame(Command),
        ))
        Guard = Transport.X660GuardedMotionTransport(
            Bus=Bus,
            DryRun=False,
            IUnderstandMotionWillOccur=True,
            IConfirmMotionAreaIsClear=True,
            TimeoutSec=0.02,
        )
        Result = Guard.Transact(
            MotionProtocol.BuildSpeedControlRequest(1.0)
        )
        self.assertEqual(Result.ReplyData[0], Command)
        self.assertEqual(len(Bus.ReceiveTimeouts), 4)

    def test_matching_reply_with_wrong_dlc_is_rejected(self):
        Command = MotionProtocol.INCREMENTAL_POSITION_COMMAND
        Bus = FakeBus((ReplyFrame(Command, Data=bytes([Command]) + bytes(6)),))
        Guard = Transport.X660GuardedMotionTransport(
            Bus=Bus,
            DryRun=False,
            IUnderstandMotionWillOccur=True,
            IConfirmMotionAreaIsClear=True,
            TimeoutSec=0.02,
        )
        with self.assertRaisesRegex(ValueError, "DLC must be 8"):
            Guard.Transact(
                MotionProtocol.BuildIncrementalPositionRequest(1.0, 2)
            )

    def test_missing_reply_times_out(self):
        Guard = Transport.X660GuardedMotionTransport(
            Bus=FakeBus(),
            DryRun=False,
            IUnderstandMotionWillOccur=True,
            IConfirmMotionAreaIsClear=True,
            TimeoutSec=0.01,
        )
        with self.assertRaisesRegex(TimeoutError, "0x81"):
            Guard.Transact(MotionProtocol.BuildMotorStopRequest())

    def test_message_factory_cannot_change_safety_critical_fields(self):
        Payload = MotionProtocol.BuildMotorStopRequest()
        Factories = (
            lambda **Fields: Transport.CanFrame(
                Fields["arbitration_id"] + 1,
                Fields["data"],
                Fields["is_extended_id"],
            ),
            lambda **Fields: Transport.CanFrame(
                Fields["arbitration_id"],
                Fields["data"],
                True,
            ),
            lambda **Fields: Transport.CanFrame(
                Fields["arbitration_id"],
                bytes.fromhex("81 00 00 00 00 00 00 01"),
                Fields["is_extended_id"],
            ),
        )
        for Factory in Factories:
            with self.subTest(Factory=Factory):
                Guard = Transport.X660GuardedMotionTransport(
                    MessageFactory=Factory
                )
                with self.assertRaises(RuntimeError):
                    Guard.Transact(Payload)

    def test_transport_has_no_can_opener_or_hardware_dependency(self):
        Source = Path(Transport.__file__).read_text(encoding="utf-8")
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
        self.assertNotIn("interface.Bus", Source)
        self.assertNotIn("socketcan", Source.lower())

    def test_scheduler_facing_controller_and_factory_remain_motion_locked(self):
        self.assertFalse(X660ReadOnlyController.MotionCommandsEnabled)
        with self.assertRaisesRegex(ValueError, "Unknown X660Mode"):
            CreateX660Controller({"X660Mode": "x660-motion"})


if __name__ == "__main__":
    unittest.main(verbosity=2)
