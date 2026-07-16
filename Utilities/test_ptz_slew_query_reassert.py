#!/usr/bin/env python3
"""
test_ptz_slew_query_reassert.py

Pelco-D PTZ test rig for Vanguard X.

Purpose:
    Prove whether querying PTZ pan position while slewing causes the PTZ
    to slow/stop, and whether reasserting the slew command after each query
    keeps the scan moving.

Test modes:
    1 = slew once, query position repeatedly
    2 = slew once, query position, then reassert same slew after each query
    3 = ping-pong scan between StartDeg and StopDeg using query + reassert

Expected:
    If mode 1 stalls/slows but mode 2 works, the PTZ requires slew reassertion
    after position query.
"""

import serial
import time
import argparse


Port = "/dev/ttyACM0"
BaudRate = 2400
Address = 1

PanSpeed = 0x5F
TiltSpeed = 0x3F

DefaultStartDeg = 60.0
DefaultStopDeg = 120.0

QueryIntervalSec = 0.20
DurationSec = 30.0


def SendPelcoD(Ser, Command1, Command2, Data1, Data2, ReadReply=True, Label=""):
    Checksum = (Address + Command1 + Command2 + Data1 + Data2) & 0xFF
    Packet = bytes([0xFF, Address, Command1, Command2, Data1, Data2, Checksum])

    if ReadReply:
        try:
            Ser.reset_input_buffer()
        except Exception:
            pass

    try:
        Ser.write(Packet)
        Ser.flush()
    except Exception as Error:
        print(f"{Label} TX ERROR:", Error)
        return b""

    if Label:
        print(f"{Label} TX:", Packet.hex(" "))

    if not ReadReply:
        return b""

    time.sleep(0.05)

    try:
        Reply = Ser.read(64)
    except Exception as Error:
        print(f"{Label} RX ERROR:", Error)
        return b""

    if Label:
        if Reply:
            print(f"{Label} RX:", Reply.hex(" "))
        else:
            print(f"{Label} RX: no reply")

    return Reply


def AngleDeltaDeg(CurrentDeg, PreviousDeg):
    Delta = CurrentDeg - PreviousDeg

    while Delta > 180.0:
        Delta -= 360.0

    while Delta < -180.0:
        Delta += 360.0

    return Delta


def ParsePanReply(Reply):
    if len(Reply) < 7:
        return None

    for Start in range(0, len(Reply) - 6):
        Frame = Reply[Start:Start + 7]

        if Frame[0] != 0xFF:
            continue

        if Frame[1] != Address:
            continue

        if Frame[3] != 0x59:
            continue

        Checksum = sum(Frame[1:6]) & 0xFF
        if Frame[6] != Checksum:
            continue

        Raw = (Frame[4] << 8) | Frame[5]
        return Raw / 100.0

    return None


def QueryPanPosition(Ser, Verbose=False):
    Reply = SendPelcoD(
        Ser,
        0x00,
        0x51,
        0x00,
        0x00,
        ReadReply=True,
        Label="QUERY" if Verbose else "",
    )
    return ParsePanReply(Reply)


def PanRight(Ser, ForceLabel="RIGHT"):
    SendPelcoD(Ser, 0x00, 0x02, PanSpeed, 0x00, ReadReply=False, Label=ForceLabel)


def PanLeft(Ser, ForceLabel="LEFT"):
    SendPelcoD(Ser, 0x00, 0x04, PanSpeed, 0x00, ReadReply=False, Label=ForceLabel)


def Stop(Ser):
    for _ in range(3):
        SendPelcoD(Ser, 0x00, 0x00, 0x00, 0x00, ReadReply=False, Label="STOP")
        time.sleep(0.04)


def CommandDirection(Ser, Direction, Label="SLEW"):
    if Direction == "right":
        PanRight(Ser, Label)
    elif Direction == "left":
        PanLeft(Ser, Label)
    else:
        Stop(Ser)


def DirectionToTarget(CurrentDeg, TargetDeg):
    if TargetDeg > CurrentDeg:
        return "right"
    if TargetDeg < CurrentDeg:
        return "left"
    return "stop"


def PrintSample(PanDeg, PreviousPanDeg, PreviousTime, Direction):
    Now = time.time()

    if PanDeg is None:
        print("PAN unavailable")
        return None, None

    if PreviousPanDeg is None or PreviousTime is None:
        print(f"PAN={PanDeg:7.2f} deg | waiting for next sample | dir={Direction}")
        return PanDeg, Now

    Dt = Now - PreviousTime
    DAz = AngleDeltaDeg(PanDeg, PreviousPanDeg)
    Rate = DAz / Dt if Dt > 0.0 else 0.0

    print(
        f"PAN={PanDeg:7.2f} deg | "
        f"dAZ={DAz:7.2f} deg | "
        f"dt={Dt:5.3f} s | "
        f"rate={Rate:7.2f} deg/s | "
        f"dir={Direction}"
    )

    return PanDeg, Now


def TestSlewQuery(Ser, Direction, DurationSec, QueryIntervalSec, ReassertAfterQuery):
    print()
    print("TEST:", "query + reassert" if ReassertAfterQuery else "query only")
    print(f"Direction={Direction}, duration={DurationSec}s, query interval={QueryIntervalSec}s")
    print()

    Stop(Ser)
    time.sleep(0.2)

    CommandDirection(Ser, Direction, "START")

    PreviousPanDeg = None
    PreviousTime = None
    StartTime = time.time()
    LastQueryTime = 0.0

    try:
        while time.time() - StartTime < DurationSec:
            Now = time.time()

            if Now - LastQueryTime >= QueryIntervalSec:
                LastQueryTime = Now

                PanDeg = QueryPanPosition(Ser)
                PreviousPanDeg, PreviousTime = PrintSample(
                    PanDeg,
                    PreviousPanDeg,
                    PreviousTime,
                    Direction,
                )

                if ReassertAfterQuery:
                    CommandDirection(Ser, Direction, "REASSERT")

            time.sleep(0.01)

    finally:
        Stop(Ser)


def TestPingPongScan(Ser, StartDeg, StopDeg, DurationSec, QueryIntervalSec, ReassertAfterQuery):
    print()
    print("TEST: ping-pong scan")
    print(f"Start={StartDeg:.2f} deg, Stop={StopDeg:.2f} deg")
    print(f"Reassert after query={ReassertAfterQuery}")
    print()

    Stop(Ser)
    time.sleep(0.2)

    PanDeg = QueryPanPosition(Ser)
    if PanDeg is None:
        print("Initial pan query failed")
        return

    TargetDeg = StopDeg
    Direction = DirectionToTarget(PanDeg, TargetDeg)

    print(f"Initial PAN={PanDeg:.2f}, first target={TargetDeg:.2f}, direction={Direction}")
    CommandDirection(Ser, Direction, "START")

    PreviousPanDeg = None
    PreviousTime = None
    StartTime = time.time()
    LastQueryTime = 0.0

    try:
        while time.time() - StartTime < DurationSec:
            Now = time.time()

            if Now - LastQueryTime >= QueryIntervalSec:
                LastQueryTime = Now

                PanDeg = QueryPanPosition(Ser)
                PreviousPanDeg, PreviousTime = PrintSample(
                    PanDeg,
                    PreviousPanDeg,
                    PreviousTime,
                    Direction,
                )

                if PanDeg is not None:
                    if Direction == "right" and PanDeg >= StopDeg:
                        print("Reached stop endpoint, reverse left")
                        TargetDeg = StartDeg
                        Direction = "left"
                        CommandDirection(Ser, Direction, "REV")

                    elif Direction == "left" and PanDeg <= StartDeg:
                        print("Reached start endpoint, reverse right")
                        TargetDeg = StopDeg
                        Direction = "right"
                        CommandDirection(Ser, Direction, "REV")

                    elif ReassertAfterQuery:
                        CommandDirection(Ser, Direction, "REASSERT")

            time.sleep(0.01)

    finally:
        Stop(Ser)


def Main():
    Parser = argparse.ArgumentParser()
    Parser.add_argument("--port", default=Port)
    Parser.add_argument("--baud", type=int, default=BaudRate)
    Parser.add_argument("--duration", type=float, default=DurationSec)
    Parser.add_argument("--query-interval", type=float, default=QueryIntervalSec)
    Parser.add_argument("--start", type=float, default=DefaultStartDeg)
    Parser.add_argument("--stop", type=float, default=DefaultStopDeg)
    Args = Parser.parse_args()

    with serial.Serial(
        Args.port,
        Args.baud,
        bytesize=8,
        parity="N",
        stopbits=1,
        timeout=0.25,
    ) as Ser:

        print("Opened", Ser.name)
        print("Baud:", Args.baud)
        print()
        print("Menu")
        print("1 = pan right, query only")
        print("2 = pan right, query + reassert")
        print("3 = ping-pong scan, query only")
        print("4 = ping-pong scan, query + reassert")
        print("5 = query position")
        print("6 = stop")
        print("q = quit")

        try:
            while True:
                Cmd = input("PTZ_TEST> ").strip().lower()

                if Cmd == "1":
                    TestSlewQuery(
                        Ser,
                        "right",
                        Args.duration,
                        Args.query_interval,
                        ReassertAfterQuery=False,
                    )

                elif Cmd == "2":
                    TestSlewQuery(
                        Ser,
                        "right",
                        Args.duration,
                        Args.query_interval,
                        ReassertAfterQuery=True,
                    )

                elif Cmd == "3":
                    TestPingPongScan(
                        Ser,
                        Args.start,
                        Args.stop,
                        Args.duration,
                        Args.query_interval,
                        ReassertAfterQuery=False,
                    )

                elif Cmd == "4":
                    TestPingPongScan(
                        Ser,
                        Args.start,
                        Args.stop,
                        Args.duration,
                        Args.query_interval,
                        ReassertAfterQuery=True,
                    )

                elif Cmd == "5":
                    PanDeg = QueryPanPosition(Ser, Verbose=True)
                    print("PAN:", PanDeg)

                elif Cmd == "6":
                    Stop(Ser)

                elif Cmd == "q":
                    break

                else:
                    print("Unknown command")

        finally:
            Stop(Ser)
            print("Done")


if __name__ == "__main__":
    Main()
