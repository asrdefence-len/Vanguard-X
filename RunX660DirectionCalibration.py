#!/usr/bin/env python3
"""Stage 4C one-move X6-60 direction calibration harness.

The default path is a dry run and does not open SocketCAN.  The armed path
performs one absolute raw-angle move of exactly +1.00 degree at 1 degree/s.
It does not alter motor configuration, encoder zero, CAN settings, or the
Vanguard X scheduler.
"""

from __future__ import annotations

import argparse
import math
import subprocess
import sys
import time

from X660CalibrationProtocol import (
    ABSOLUTE_POSITION_COMMAND,
    MOTOR_STOP_COMMAND,
    BuildAbsolutePositionRequest,
    BuildMotorStopRequest,
    ValidateMotionReply,
)


CALIBRATION_DELTA_DEG = 1.00
CALIBRATION_SPEED_DEG_PER_SEC = 1
MAX_TRAVEL_FROM_START_DEG = 1.50
MAX_ABS_CURRENT_A = 2.00
MAX_TEMPERATURE_C = 50
TARGET_TOLERANCE_DEG = 0.15
REVERSE_MOTION_GUARD_DEG = 0.20
SETTLE_SAMPLES = 3
POLL_PERIOD_SEC = 0.05
MOVE_TIMEOUT_SEC = 5.0


def ParseArguments(Argv=None):
    Parser = argparse.ArgumentParser(
        description="One-degree Stage 4C X6-60 raw-direction calibration"
    )
    Parser.add_argument("--interface", default="can0")
    Parser.add_argument("--node-id", type=int, default=1)
    Parser.add_argument("--timeout-ms", type=float, default=250.0)
    Parser.add_argument("--execute-one-degree-calibration", action="store_true")
    Parser.add_argument(
        "--i-confirm-x6-60-is-secured-and-area-clear", action="store_true"
    )
    Parser.add_argument(
        "--i-confirm-at-least-two-degrees-clearance-in-both-directions",
        action="store_true",
    )
    Parser.add_argument(
        "--i-confirm-power-isolation-is-immediately-accessible",
        action="store_true",
    )
    return Parser.parse_args(Argv)


def ValidateArguments(Arguments):
    if not 1 <= int(Arguments.node_id) <= 32:
        raise ValueError("--node-id must be between 1 and 32")
    if not Arguments.interface or any(
        Character.isspace() for Character in Arguments.interface
    ):
        raise ValueError("--interface must be one non-empty device name")
    if not 10.0 <= float(Arguments.timeout_ms) <= 1000.0:
        raise ValueError("--timeout-ms must be between 10 and 1000")

    if Arguments.execute_one_degree_calibration:
        Required = (
            (
                Arguments.i_confirm_x6_60_is_secured_and_area_clear,
                "--i-confirm-x6-60-is-secured-and-area-clear",
            ),
            (
                Arguments.i_confirm_at_least_two_degrees_clearance_in_both_directions,
                "--i-confirm-at-least-two-degrees-clearance-in-both-directions",
            ),
            (
                Arguments.i_confirm_power_isolation_is_immediately_accessible,
                "--i-confirm-power-isolation-is-immediately-accessible",
            ),
        )
        Missing = [Name for Present, Name in Required if not Present]
        if Missing:
            raise ValueError("Missing safety acknowledgement(s): " + ", ".join(Missing))


def DescribePlan(Arguments):
    print("Vanguard X Stage 4C X6-60 direction calibration")
    print(f"  interface/node:   {Arguments.interface} / {Arguments.node_id}")
    print(f"  commanded change: raw angle +{CALIBRATION_DELTA_DEG:.2f} deg")
    print(f"  speed limit:      {CALIBRATION_SPEED_DEG_PER_SEC} deg/s")
    print(f"  travel guard:     {MAX_TRAVEL_FROM_START_DEG:.2f} deg from start")
    print(f"  current guard:    {MAX_ABS_CURRENT_A:.2f} A")
    print("  scheduler:        NOT USED; remains motion-locked")
    print("  configuration:    no writes; encoder zero unchanged")


def CheckCanInterface(Interface: str):
    Result = subprocess.run(
        ["ip", "-details", "link", "show", "dev", Interface],
        capture_output=True,
        text=True,
        check=False,
    )
    if Result.returncode != 0:
        raise RuntimeError(
            f"CAN interface {Interface!r} was not found: {Result.stderr.strip()}"
        )
    if "state UP" not in Result.stdout:
        raise RuntimeError(f"CAN interface {Interface!r} is not UP")
    if "bitrate 1000000" not in Result.stdout:
        raise RuntimeError(f"CAN interface {Interface!r} is not reporting 1 Mbit/s")


def DrainReceiveQueue(Bus, MaximumFrames=1000):
    Count = 0
    while Count < int(MaximumFrames):
        if Bus.recv(timeout=0.0) is None:
            break
        Count += 1
    if Count >= int(MaximumFrames):
        raise RuntimeError("CAN receive queue did not drain")
    return Count


def ReceiveReply(Bus, Protocol, NodeId, Command, TimeoutSec):
    Deadline = time.monotonic() + TimeoutSec
    while True:
        Remaining = Deadline - time.monotonic()
        if Remaining <= 0.0:
            raise TimeoutError(f"Timed out waiting for X6-60 command 0x{Command:02X}")
        Reply = Bus.recv(timeout=Remaining)
        if Reply is None:
            continue
        if (
            int(Reply.arbitration_id) != Protocol.ReplyArbitrationId(NodeId)
            or bool(Reply.is_extended_id)
        ):
            continue
        return Reply


def TransactRead(Bus, CanModule, Protocol, NodeId, Command, TimeoutSec):
    Bus.send(
        CanModule.Message(
            arbitration_id=Protocol.RequestArbitrationId(NodeId),
            data=Protocol.BuildReadRequest(Command),
            is_extended_id=False,
        ),
        timeout=TimeoutSec,
    )
    Reply = ReceiveReply(Bus, Protocol, NodeId, Command, TimeoutSec)
    return Protocol.ValidateReply(
        NodeId, Command, Reply.arbitration_id, Reply.data, Reply.is_extended_id
    )


def TransactMotion(Bus, CanModule, Protocol, NodeId, FrameData, TimeoutSec):
    Command = int(FrameData[0])
    Bus.send(
        CanModule.Message(
            arbitration_id=Protocol.RequestArbitrationId(NodeId),
            data=FrameData,
            is_extended_id=False,
        ),
        timeout=TimeoutSec,
    )
    Reply = ReceiveReply(Bus, Protocol, NodeId, Command, TimeoutSec)
    return ValidateMotionReply(Command, Reply.data)


def AttemptEmergencyStop(Bus, CanModule, Protocol, NodeId, TimeoutSec):
    try:
        TransactMotion(
            Bus,
            CanModule,
            Protocol,
            NodeId,
            BuildMotorStopRequest(),
            TimeoutSec,
        )
        print("  SAFEGUARD:        0x81 motor-stop acknowledged", file=sys.stderr)
    except Exception as Error:
        print(f"  SAFEGUARD FAILED: motor-stop not acknowledged: {Error}", file=sys.stderr)


def ReadMotionState(Bus, CanModule, Protocol, NodeId, TimeoutSec):
    RawAngle = Protocol.DecodeMultiTurnAngleDeg(
        TransactRead(
            Bus, CanModule, Protocol, NodeId, Protocol.READ_MULTI_TURN_ANGLE, TimeoutSec
        )
    )
    Status2 = Protocol.DecodeStatus2(
        TransactRead(Bus, CanModule, Protocol, NodeId, Protocol.READ_STATUS_2, TimeoutSec)
    )
    return float(RawAngle), Status2


def ExecuteCalibration(Arguments):
    try:
        import can
        import X660CanProtocol as Protocol
    except Exception as Error:
        raise RuntimeError(
            "Stage 4C requires python-can and the proven Stage 4A X660CanProtocol.py"
        ) from Error

    CheckCanInterface(Arguments.interface)
    Protocol.ValidateNodeId(Arguments.node_id)
    TimeoutSec = float(Arguments.timeout_ms) / 1000.0
    Bus = can.interface.Bus(channel=Arguments.interface, interface="socketcan")
    MotionCommandSent = False
    ReachedTarget = False
    try:
        Drained = DrainReceiveQueue(Bus)
        Status1 = Protocol.DecodeStatus1(
            TransactRead(
                Bus, can, Protocol, Arguments.node_id, Protocol.READ_STATUS_1, TimeoutSec
            )
        )
        StartRawAngle, StartStatus2 = ReadMotionState(
            Bus, can, Protocol, Arguments.node_id, TimeoutSec
        )
        print(f"  stale frames:     {Drained}")
        print(
            f"  preflight:        raw={StartRawAngle:+.2f} deg, "
            f"speed={StartStatus2.SpeedDegPerSec:+d} deg/s, "
            f"current={StartStatus2.TorqueCurrentA:+.2f} A, "
            f"temp={Status1.TemperatureC} C, errors=0x{Status1.ErrorFlags:04X}"
        )

        if Status1.ErrorFlags:
            raise RuntimeError("preflight failed: motor reports error flags")
        if Status1.TemperatureC > MAX_TEMPERATURE_C:
            raise RuntimeError("preflight failed: motor temperature exceeds limit")
        if abs(float(StartStatus2.SpeedDegPerSec)) > 0.0:
            raise RuntimeError("preflight failed: motor is not stationary")
        if abs(float(StartStatus2.TorqueCurrentA)) > 0.25:
            raise RuntimeError("preflight failed: unexpected stationary current")
        if not math.isfinite(StartRawAngle):
            raise RuntimeError("preflight failed: non-finite raw angle")

        TargetRawAngle = StartRawAngle + CALIBRATION_DELTA_DEG
        Frame = BuildAbsolutePositionRequest(
            TargetRawAngle, CALIBRATION_SPEED_DEG_PER_SEC
        )
        print(f"  target raw angle: {TargetRawAngle:+.2f} deg")
        print("  COMMAND:          sending one 0xA4 absolute-position frame")
        TransactMotion(
            Bus, can, Protocol, Arguments.node_id, Frame, TimeoutSec
        )
        MotionCommandSent = True

        Deadline = time.monotonic() + MOVE_TIMEOUT_SEC
        Settled = 0
        SampleIndex = 0
        while time.monotonic() < Deadline:
            RawAngle, Status2 = ReadMotionState(
                Bus, can, Protocol, Arguments.node_id, TimeoutSec
            )
            SampleIndex += 1
            Travel = RawAngle - StartRawAngle
            Error = TargetRawAngle - RawAngle
            print(
                f"  sample {SampleIndex:02d}:       raw={RawAngle:+.2f} deg, "
                f"travel={Travel:+.2f} deg, speed={Status2.SpeedDegPerSec:+d} deg/s, "
                f"current={Status2.TorqueCurrentA:+.2f} A"
            )

            if abs(Travel) > MAX_TRAVEL_FROM_START_DEG:
                raise RuntimeError("travel guard exceeded")
            if Travel < -REVERSE_MOTION_GUARD_DEG:
                raise RuntimeError("raw angle moved opposite the commanded direction")
            if abs(float(Status2.TorqueCurrentA)) > MAX_ABS_CURRENT_A:
                raise RuntimeError("current guard exceeded")

            if abs(Error) <= TARGET_TOLERANCE_DEG and abs(
                float(Status2.SpeedDegPerSec)
            ) <= 1.0:
                Settled += 1
                if Settled >= SETTLE_SAMPLES:
                    ReachedTarget = True
                    break
            else:
                Settled = 0
            time.sleep(POLL_PERIOD_SEC)

        if not ReachedTarget:
            raise TimeoutError("one-degree target did not settle before timeout")

        FinalStatus1 = Protocol.DecodeStatus1(
            TransactRead(
                Bus, can, Protocol, Arguments.node_id, Protocol.READ_STATUS_1, TimeoutSec
            )
        )
        FinalRawAngle, FinalStatus2 = ReadMotionState(
            Bus, can, Protocol, Arguments.node_id, TimeoutSec
        )
        if FinalStatus1.ErrorFlags:
            raise RuntimeError("motor reports error flags after calibration move")

        print(
            f"PASS: raw angle increased by {FinalRawAngle - StartRawAngle:+.2f} deg "
            "for the one-degree calibration command"
        )
        print(
            "REPORT THE PHYSICAL DIRECTION: did the antenna move clockwise/right "
            "or anticlockwise/left when viewed from above?"
        )
        print(
            f"  final:            raw={FinalRawAngle:+.2f} deg, "
            f"speed={FinalStatus2.SpeedDegPerSec:+d} deg/s, "
            f"current={FinalStatus2.TorqueCurrentA:+.2f} A"
        )
    except Exception:
        if MotionCommandSent and not ReachedTarget:
            AttemptEmergencyStop(
                Bus, can, Protocol, Arguments.node_id, TimeoutSec
            )
        raise
    finally:
        Bus.shutdown()


def Main(Argv=None):
    Arguments = ParseArguments(Argv)
    ValidateArguments(Arguments)
    DescribePlan(Arguments)
    if not Arguments.execute_one_degree_calibration:
        print("DRY RUN: no CAN interface opened and no frame transmitted")
        return 0
    ExecuteCalibration(Arguments)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(Main())
    except Exception as Error:
        print(f"FAIL: {Error}", file=sys.stderr)
        sys.exit(1)
