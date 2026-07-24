#!/usr/bin/env python3
"""Software-only tests for fail-safe X6-60 planner initialisation."""

from collections import deque
import unittest

from X660GuardedMotionTransport import CanFrame
import X660PlannerInitialiser as Initialiser
import X660PlannerProtocol as Protocol


class FakePlannerBus:
    def __init__(self, Values, IgnoreWriteIndices=(), TimeoutIndices=()):
        self.Values = list(Values)
        self.IgnoreWriteIndices = set(IgnoreWriteIndices)
        self.TimeoutIndices = set(TimeoutIndices)
        self.Requests = []
        self.Replies = deque()

    def send(self, Request, timeout):
        Data = bytes(Request.data)
        self.Requests.append(Data)
        Command = Data[0]
        Index = Data[1]
        if (Command, Index) in self.TimeoutIndices:
            return
        if Command == Protocol.READ_PLANNER_PARAMETER:
            Value = self.Values[Index]
            ReplyData = bytes([Command, Index, 0, 0]) + int(Value).to_bytes(
                4,
                "little",
            )
        elif Command == Protocol.WRITE_PLANNER_PARAMETER:
            Value = int.from_bytes(Data[4:8], "little")
            if Index not in self.IgnoreWriteIndices:
                self.Values[Index] = Value
            ReplyData = Data
        else:
            raise AssertionError(f"unexpected command 0x{Command:02X}")
        self.Replies.append(
            CanFrame(
                arbitration_id=0x241,
                data=ReplyData,
                is_extended_id=False,
            )
        )

    def recv(self, timeout):
        return self.Replies.popleft() if self.Replies else None


def MakeInitialiser(Bus):
    return Initialiser.X660PlannerInitialiser(
        NodeId=1,
        Bus=Bus,
        MessageFactory=CanFrame,
        TimeoutSec=0.01,
        RequiredValues=(1140, 1140, 1140, 1140),
    )


class TestX660PlannerInitialiser(unittest.TestCase):
    def test_matching_values_are_verified_without_rom_writes(self):
        Bus = FakePlannerBus((1140, 1140, 1140, 1140))
        Result = MakeInitialiser(Bus).Initialise()
        self.assertEqual(Result.Values, (1140, 1140, 1140, 1140))
        self.assertEqual(Result.CorrectedIndices, ())
        self.assertEqual(Result.WriteCount, 0)
        self.assertFalse(any(Data[0] == 0x43 for Data in Bus.Requests))

    def test_only_mismatched_indices_are_written_then_all_are_verified(self):
        Bus = FakePlannerBus((100, 100, 1140, 1140))
        Result = MakeInitialiser(Bus).Initialise()
        Writes = [Data for Data in Bus.Requests if Data[0] == 0x43]
        self.assertEqual(
            Writes,
            [
                bytes.fromhex("43 00 00 00 74 04 00 00"),
                bytes.fromhex("43 01 00 00 74 04 00 00"),
            ],
        )
        self.assertEqual(Result.CorrectedIndices, (0, 1))
        self.assertEqual(Result.WriteCount, 2)
        self.assertEqual(Result.Values, (1140, 1140, 1140, 1140))

    def test_failed_readback_prevents_motor_ready_result(self):
        Bus = FakePlannerBus(
            (100, 1140, 1140, 1140),
            IgnoreWriteIndices=(0,),
        )
        with self.assertRaisesRegex(RuntimeError, "immediate verification"):
            MakeInitialiser(Bus).Initialise()

    def test_timeout_prevents_motor_ready_result(self):
        Bus = FakePlannerBus(
            (1140, 1140, 1140, 1140),
            TimeoutIndices=((0x42, 2),),
        )
        with self.assertRaisesRegex(TimeoutError, "index 0x02"):
            MakeInitialiser(Bus).Initialise()


if __name__ == "__main__":
    unittest.main()
