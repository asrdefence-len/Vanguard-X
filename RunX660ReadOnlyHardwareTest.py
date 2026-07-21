#!/usr/bin/env python3
"""Stage 4A read-only X6-60 CAN hardware test harness.

No command capable of motion or configuration is implemented in this file.
The harness does not bring up or modify the host CAN interface.
"""

from __future__ import annotations

import argparse
import math
import subprocess
import sys
import time

from X660CanProtocol import (
    BuildReadRequest,
    DecodeMultiTurnAngleDeg,
    DecodeSoftwareVersion,
    DecodeStatus1,
    DecodeStatus2,
    READ_MULTI_TURN_ANGLE,
    READ_SOFTWARE_VERSION,
    READ_STATUS_1,
    READ_STATUS_2,
    ReplyArbitrationId,
    RequestArbitrationId,
    ValidateNodeId,
    ValidateReply,
)


READ_SEQUENCE = (
    READ_SOFTWARE_VERSION,
    READ_STATUS_1,
    READ_MULTI_TURN_ANGLE,
    READ_STATUS_2,
)


def ParseArguments(Argv=None):
    Parser = argparse.ArgumentParser(
        description="Read-only Stage 4A X6-60 CAN verification",
    )
    Parser.add_argument("--interface", default="can0")
    Parser.add_argument("--node-id", type=int, default=1)
    Parser.add_argument("--timeout-ms", type=float, default=250.0)
    Parser.add_argument("--samples", type=int, default=10)
    Parser.add_argument("--period-ms", type=float, default=100.0)
    Parser.add_argument(
        "--max-stationary-speed-deg-s",
        type=float,
        default=1.0,
    )
    Parser.add_argument(
        "--execute-read-only",
        action="store_true",
        help="open SocketCAN and transmit only the four documented read requests",
    )
    Parser.add_argument(
        "--i-confirm-x6-60-is-secured-and-area-clear",
        action="store_true",
    )
    Parser.add_argument(
        "--i-confirm-no-motion-or-configuration-commands",
        action="store_true",
    )
    return Parser.parse_args(Argv)


def ValidateArguments(Arguments):
    ValidateNodeId(Arguments.node_id)
    if not Arguments.interface or any(
        Character.isspace() for Character in Arguments.interface
    ):
        raise ValueError("CAN interface must be one non-empty device name")
    if not 10.0 <= float(Arguments.timeout_ms) <= 5000.0:
        raise ValueError("--timeout-ms must be between 10 and 5000")
    if not 1 <= int(Arguments.samples) <= 100:
        raise ValueError("--samples must be between 1 and 100")
    if not 20.0 <= float(Arguments.period_ms) <= 5000.0:
        raise ValueError("--period-ms must be between 20 and 5000")
    if not 0.0 <= float(Arguments.max_stationary_speed_deg_s) <= 10.0:
        raise ValueError(
            "--max-stationary-speed-deg-s must be between 0 and 10"
        )
    if Arguments.execute_read_only:
        if not Arguments.i_confirm_x6_60_is_secured_and_area_clear:
            raise ValueError(
                "Missing --i-confirm-x6-60-is-secured-and-area-clear"
            )
        if not Arguments.i_confirm_no_motion_or_configuration_commands:
            raise ValueError(
                "Missing --i-confirm-no-motion-or-configuration-commands"
            )


def DescribePlan(Arguments):
    NodeId = ValidateNodeId(Arguments.node_id)
    print("Vanguard X Stage 4A X6-60 read-only verification")
    print(f"  interface:        {Arguments.interface} (must already be UP at 1 Mbit/s)")
    print(f"  node:             {NodeId}")
    print(f"  request/reply ID: 0x{RequestArbitrationId(NodeId):03X} / "
          f"0x{ReplyArbitrationId(NodeId):03X}")
    print("  permitted reads:  0xB2 version, 0x9A status/error, "
          "0x92 multi-turn angle, 0x9C speed/current")
    print("  motion commands:  NOT IMPLEMENTED")
    print("  configuration:    NOT IMPLEMENTED")
    print("  automatic stop:   NOT SENT (could unexpectedly release holding torque)")


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
    Text = Result.stdout
    if "state UP" not in Text:
        raise RuntimeError(f"CAN interface {Interface!r} is not UP")
    if "bitrate 1000000" not in Text:
        raise RuntimeError(
            f"CAN interface {Interface!r} is not reporting 1 Mbit/s"
        )


def DrainReceiveQueue(Bus, MaximumFrames=1000):
    Count = 0
    while Count < int(MaximumFrames):
        Message = Bus.recv(timeout=0.0)
        if Message is None:
            break
        Count += 1
    if Count >= int(MaximumFrames):
        raise RuntimeError("CAN receive queue did not drain")
    return Count


def TransactRead(Bus, CanModule, NodeId, Command, TimeoutSec):
    Request = CanModule.Message(
        arbitration_id=RequestArbitrationId(NodeId),
        data=BuildReadRequest(Command),
        is_extended_id=False,
    )
    Bus.send(Request, timeout=TimeoutSec)
    Deadline = time.monotonic() + TimeoutSec
    while True:
        Remaining = Deadline - time.monotonic()
        if Remaining <= 0.0:
            raise TimeoutError(
                f"Timed out waiting for X6-60 command 0x{Command:02X}"
            )
        Reply = Bus.recv(timeout=Remaining)
        if Reply is None:
            continue
        if (
            int(Reply.arbitration_id) != ReplyArbitrationId(NodeId)
            or bool(Reply.is_extended_id)
        ):
            continue
        return ValidateReply(
            NodeId,
            Command,
            Reply.arbitration_id,
            Reply.data,
            Reply.is_extended_id,
        )


def ExecuteReadOnlyTest(Arguments):
    try:
        import can
    except Exception as Error:
        raise RuntimeError(
            "python-can is unavailable; install it with "
            "'python3 -m pip install python-can'"
        ) from Error

    CheckCanInterface(Arguments.interface)
    TimeoutSec = float(Arguments.timeout_ms) / 1000.0
    PeriodSec = float(Arguments.period_ms) / 1000.0
    Bus = can.interface.Bus(
        channel=Arguments.interface,
        interface="socketcan",
    )
    try:
        Drained = DrainReceiveQueue(Bus)
        print(f"  stale frames:     {Drained} drained before testing")

        Version = DecodeSoftwareVersion(TransactRead(
            Bus, can, Arguments.node_id, READ_SOFTWARE_VERSION, TimeoutSec,
        ))
        Status1 = DecodeStatus1(TransactRead(
            Bus, can, Arguments.node_id, READ_STATUS_1, TimeoutSec,
        ))
        print(f"  firmware date:    {Version}")
        print(f"  bus voltage:      {Status1.BusVoltageV:.1f} V")
        print(f"  temperature:      {Status1.TemperatureC} C")
        print(f"  brake command:    "
              f"{'RELEASED' if Status1.BrakeReleased else 'LOCKED'}")
        print(f"  error flags:      0x{Status1.ErrorFlags:04X}")
        if Status1.ErrorNames:
            print("  errors:           " + ", ".join(Status1.ErrorNames))

        Angles = []
        Speeds = []
        Currents = []
        for SampleIndex in range(int(Arguments.samples)):
            Angle = DecodeMultiTurnAngleDeg(TransactRead(
                Bus, can, Arguments.node_id, READ_MULTI_TURN_ANGLE, TimeoutSec,
            ))
            Status2 = DecodeStatus2(TransactRead(
                Bus, can, Arguments.node_id, READ_STATUS_2, TimeoutSec,
            ))
            Angles.append(Angle)
            Speeds.append(float(Status2.SpeedDegPerSec))
            Currents.append(float(Status2.TorqueCurrentA))
            print(
                f"  sample {SampleIndex + 1:02d}:     "
                f"angle={Angle:+.2f} deg, "
                f"speed={Status2.SpeedDegPerSec:+d} deg/s, "
                f"current={Status2.TorqueCurrentA:+.2f} A"
            )
            if SampleIndex + 1 < int(Arguments.samples):
                time.sleep(PeriodSec)

        Failures = []
        if Status1.ErrorFlags:
            Failures.append("motor reports error flags")
        if not all(math.isfinite(Value) for Value in Angles + Speeds + Currents):
            Failures.append("non-finite telemetry")
        MaximumSpeed = max(abs(Value) for Value in Speeds)
        if MaximumSpeed > float(Arguments.max_stationary_speed_deg_s):
            Failures.append(
                f"reported speed {MaximumSpeed:.1f} deg/s exceeds stationary limit"
            )
        if Failures:
            raise RuntimeError("; ".join(Failures))
        print("PASS: X6-60 read-only CAN identification and telemetry verified")
    finally:
        Bus.shutdown()


def Main(Argv=None):
    Arguments = ParseArguments(Argv)
    ValidateArguments(Arguments)
    DescribePlan(Arguments)
    if not Arguments.execute_read_only:
        print("DRY RUN: add --execute-read-only and both acknowledgements to access CAN")
        return 0
    ExecuteReadOnlyTest(Arguments)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(Main())
    except Exception as Error:
        print(f"FAIL: {Error}", file=sys.stderr)
        sys.exit(1)
