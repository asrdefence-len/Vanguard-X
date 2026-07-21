#!/usr/bin/env python3
"""Disable X6-60 motor output, with optional explicit brake release.

Motor Motion Protocol V4.2 defines 0x80 as disabling motor output and clearing
the running/closed-loop state.  The optional 0x77 command releases the holding
brake so the shaft can move freely.  This utility never sends a motion, torque,
position, speed, reset, or configuration command.

anguard@vanguard-AG2:~/Projects/Software$ cd ~/Projects/Software

python3 RunX660MotorShutdown.py \
    --interface can0 \
    --node-id 1 \
    --i-confirm-axis-is-supported \
    --i-understand-motor-output-will-be-disabled
X6-60 motor output disabled; closed-loop running state cleared
Status: temperature=53 C, brake_released=False, bus_voltage=48.6 V, error_flags=0x0000
vanguard@vanguard-AG2:~/Projects/Software$ 

"""

from __future__ import annotations

import argparse
import struct
import subprocess
import time


MOTOR_SHUTDOWN_COMMAND = 0x80
BRAKE_RELEASE_COMMAND = 0x77
READ_STATUS_1_COMMAND = 0x9A
CAN_DLC = 8


def RequestArbitrationId(NodeId: int) -> int:
    return 0x140 + int(NodeId)


def ReplyArbitrationId(NodeId: int) -> int:
    return 0x240 + int(NodeId)


def BuildEmptyCommand(Command: int) -> bytes:
    if int(Command) not in (
        MOTOR_SHUTDOWN_COMMAND,
        BRAKE_RELEASE_COMMAND,
        READ_STATUS_1_COMMAND,
    ):
        raise ValueError("command is outside this utility's allow-list")
    return bytes([int(Command)]) + bytes(7)


def CheckCanInterface(Interface: str) -> None:
    Result = subprocess.run(
        ["ip", "-details", "link", "show", "dev", Interface],
        capture_output=True,
        text=True,
        check=False,
    )
    if Result.returncode != 0:
        raise RuntimeError(
            f"CAN interface {Interface!r} was not found: "
            f"{Result.stderr.strip()}"
        )
    if "state UP" not in Result.stdout:
        raise RuntimeError(f"CAN interface {Interface!r} is not UP")
    if "bitrate 1000000" not in Result.stdout:
        raise RuntimeError(
            f"CAN interface {Interface!r} is not reporting 1 Mbit/s"
        )


def DrainReceiveQueue(Bus, MaximumFrames: int = 1000) -> None:
    for _ in range(int(MaximumFrames)):
        if Bus.recv(timeout=0.0) is None:
            return
    raise RuntimeError("CAN receive queue did not drain")


def Transact(Bus, CanModule, NodeId: int, Payload: bytes, TimeoutSec: float) -> bytes:
    Command = Payload[0]
    Bus.send(
        CanModule.Message(
            arbitration_id=RequestArbitrationId(NodeId),
            data=Payload,
            is_extended_id=False,
        ),
        timeout=TimeoutSec,
    )
    Deadline = time.monotonic() + TimeoutSec
    while True:
        Remaining = Deadline - time.monotonic()
        if Remaining <= 0.0:
            raise TimeoutError(
                f"timed out waiting for X6-60 command 0x{Command:02X}"
            )
        Reply = Bus.recv(timeout=Remaining)
        if Reply is None:
            continue
        if (
            int(Reply.arbitration_id) != ReplyArbitrationId(NodeId)
            or bool(Reply.is_extended_id)
        ):
            continue
        Data = bytes(Reply.data)
        if len(Data) != CAN_DLC:
            raise RuntimeError(
                f"reply DLC must be 8; received {len(Data)}"
            )
        if Data[0] != Command:
            continue
        return Data


def ParseArguments():
    Parser = argparse.ArgumentParser(description=__doc__)
    Parser.add_argument("--interface", default="can0")
    Parser.add_argument("--node-id", type=int, default=1)
    Parser.add_argument("--timeout-sec", type=float, default=0.25)
    Parser.add_argument(
        "--release-brake",
        action="store_true",
        help="after shutdown, send 0x77 so the shaft can move freely",
    )
    Parser.add_argument(
        "--i-confirm-axis-is-supported",
        action="store_true",
        help="confirm the antenna/axis cannot fall, slew, or rotate dangerously",
    )
    Parser.add_argument(
        "--i-understand-motor-output-will-be-disabled",
        action="store_true",
    )
    Parser.add_argument(
        "--i-confirm-free-motion-is-safe",
        action="store_true",
        help="required only with --release-brake",
    )
    Args = Parser.parse_args()
    if not 1 <= Args.node_id <= 32:
        Parser.error("--node-id must be between 1 and 32")
    if not 0.01 <= Args.timeout_sec <= 5.0:
        Parser.error("--timeout-sec must be between 0.01 and 5.0")
    if not Args.i_confirm_axis_is_supported:
        Parser.error("--i-confirm-axis-is-supported is required")
    if not Args.i_understand_motor_output_will_be_disabled:
        Parser.error(
            "--i-understand-motor-output-will-be-disabled is required"
        )
    if Args.release_brake and not Args.i_confirm_free_motion_is_safe:
        Parser.error(
            "--release-brake requires --i-confirm-free-motion-is-safe"
        )
    return Args


def Main() -> int:
    Args = ParseArguments()
    CheckCanInterface(Args.interface)

    try:
        import can
    except Exception as Error:
        raise RuntimeError("python-can is required") from Error

    Bus = can.interface.Bus(channel=Args.interface, interface="socketcan")
    try:
        DrainReceiveQueue(Bus)
        ShutdownReply = Transact(
            Bus,
            can,
            Args.node_id,
            BuildEmptyCommand(MOTOR_SHUTDOWN_COMMAND),
            Args.timeout_sec,
        )
        if ShutdownReply != BuildEmptyCommand(MOTOR_SHUTDOWN_COMMAND):
            raise RuntimeError("0x80 shutdown reply did not echo the request")
        print("X6-60 motor output disabled; closed-loop running state cleared")

        if Args.release_brake:
            BrakeReply = Transact(
                Bus,
                can,
                Args.node_id,
                BuildEmptyCommand(BRAKE_RELEASE_COMMAND),
                Args.timeout_sec,
            )
            if BrakeReply != BuildEmptyCommand(BRAKE_RELEASE_COMMAND):
                raise RuntimeError("0x77 brake-release reply did not echo the request")
            print("X6-60 holding brake released; shaft may move freely")

        Status = Transact(
            Bus,
            can,
            Args.node_id,
            BuildEmptyCommand(READ_STATUS_1_COMMAND),
            Args.timeout_sec,
        )
        TemperatureC = struct.unpack("<b", Status[1:2])[0]
        BrakeReleased = Status[3] == 1
        BusVoltageV = int.from_bytes(Status[4:6], "little") * 0.1
        ErrorFlags = int.from_bytes(Status[6:8], "little")
        print(
            "Status: "
            f"temperature={TemperatureC} C, "
            f"brake_released={BrakeReleased}, "
            f"bus_voltage={BusVoltageV:.1f} V, "
            f"error_flags=0x{ErrorFlags:04X}"
        )
    finally:
        Bus.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(Main())
