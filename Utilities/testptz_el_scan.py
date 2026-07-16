import serial
import time

# Pelco-D EL / tilt scan test for Vanguard X PTZ
# Purpose:
#   - Read elevation using the native tilt-position query.
#   - Slew tilt up/down continuously.
#   - Estimate EL rate from position replies.
#   - Reverse when it reaches software EL limits or appears stalled at a limit switch.
#
# Notes:
#   Pan query:   00 51 -> response 00 59, raw = deg * 100
#   Tilt query:  00 53 -> response 00 5B, raw = deg * 100  (to be confirmed on this head)
#   Tilt up:     command2 bit 0x08, tilt speed in Data2
#   Tilt down:   command2 bit 0x10, tilt speed in Data2

Port = "/dev/ttyACM0"
BaudRate = 2400
Address = 1

PanSpeed = 0x5F
TiltSpeed = 0x20   # start gentler than 0x3F for limit testing

# Keep these conservative for the first test.
# If your first reading is around 82.5 and that is the upper hard stop,
# this script should reverse before repeatedly pushing into it.
LowerElLimitDeg = -10.0
UpperElLimitDeg = 82.0
LimitMarginDeg = 1.0

DurationSec = 60.0
QueryIntervalSec = 0.25

MinMovingRateDegPerSec = 0.5
StallSamplesRequired = 4

# Choose initial test direction. If the head is currently at the 82.5 deg stop,
# start DOWN so it immediately backs away from that stop.
InitialDirection = "down"   # "up" or "down"

# If the reported EL increases when the antenna physically moves down, set this True.
InvertDisplayedEl = False


def SendPelcoD(Ser, Command1, Command2, Data1, Data2, ReadReply=True):
    Checksum = (Address + Command1 + Command2 + Data1 + Data2) & 0xFF
    Packet = bytes([0xFF, Address & 0xFF, Command1 & 0xFF, Command2 & 0xFF, Data1 & 0xFF, Data2 & 0xFF, Checksum])

    if ReadReply:
        try:
            Ser.reset_input_buffer()
        except Exception:
            pass

    Ser.write(Packet)
    Ser.flush()
    print("TX:", Packet.hex(" "))

    if ReadReply:
        time.sleep(0.05)
        Reply = Ser.read(64)

        if Reply:
            print("RX:", Reply.hex(" "))
        else:
            print("RX: no reply")

        return Reply

    return b""


def ParsePelcoPositionReply(Reply, ExpectedCommand2):
    if len(Reply) < 7:
        return None

    for Start in range(0, len(Reply) - 6):
        Frame = Reply[Start:Start + 7]

        if Frame[0] != 0xFF:
            continue
        if Frame[1] != (Address & 0xFF):
            continue
        if Frame[3] != ExpectedCommand2:
            continue

        Checksum = sum(Frame[1:6]) & 0xFF
        if Frame[6] != Checksum:
            print("Bad checksum in reply frame:", Frame.hex(" "))
            continue

        Raw = (Frame[4] << 8) | Frame[5]
        return Raw / 100.0

    return None


def QueryPanPosition(Ser):
    Reply = SendPelcoD(Ser, 0x00, 0x51, 0x00, 0x00, ReadReply=True)
    return ParsePelcoPositionReply(Reply, 0x59)


def QueryTiltPosition(Ser):
    Reply = SendPelcoD(Ser, 0x00, 0x53, 0x00, 0x00, ReadReply=True)
    TiltDeg = ParsePelcoPositionReply(Reply, 0x5B)

    if TiltDeg is None:
        return None

    if InvertDisplayedEl:
        TiltDeg = -TiltDeg

    return TiltDeg


def TiltUp(Ser):
    # Pelco-D tilt up: Command2 bit 0x08, tilt speed in Data2.
    SendPelcoD(Ser, 0x00, 0x08, 0x00, TiltSpeed, ReadReply=False)


def TiltDown(Ser):
    # Pelco-D tilt down: Command2 bit 0x10, tilt speed in Data2.
    SendPelcoD(Ser, 0x00, 0x10, 0x00, TiltSpeed, ReadReply=False)


def Stop(Ser):
    # Send several stops because some Pelco-D heads keep executing motion briefly.
    for _ in range(3):
        SendPelcoD(Ser, 0x00, 0x00, 0x00, 0x00, ReadReply=False)
        time.sleep(0.04)


def CommandDirection(Ser, Direction):
    if Direction == "up":
        TiltUp(Ser)
    elif Direction == "down":
        TiltDown(Ser)
    else:
        Stop(Ser)


def ReverseDirection(Direction):
    return "down" if Direction == "up" else "up"


with serial.Serial(
    Port,
    BaudRate,
    bytesize=8,
    parity="N",
    stopbits=1,
    timeout=0.2,
) as Ser:

    print("Opened", Ser.name)
    print("Baud:", BaudRate)
    print(f"EL scan window: {LowerElLimitDeg:.2f} deg to {UpperElLimitDeg:.2f} deg")
    print(f"Initial tilt direction: {InitialDirection}")
    print("Querying initial AZ/EL...")

    InitialPan = QueryPanPosition(Ser)
    InitialEl = QueryTiltPosition(Ser)
    print(f"Initial PAN={InitialPan}, EL={InitialEl}")

    Direction = InitialDirection.lower().strip()
    if Direction not in ["up", "down"]:
        Direction = "down"

    PreviousElDeg = None
    PreviousTime = None
    StallCount = 0

    StartTime = time.time()
    LastQueryTime = 0.0
    IgnoreStallUntilTime = time.time() + 1.0

    print(f"Start tilt {Direction}")
    CommandDirection(Ser, Direction)

    try:
        while time.time() - StartTime < DurationSec:
            Now = time.time()

            if Now - LastQueryTime >= QueryIntervalSec:
                LastQueryTime = Now

                ElDeg = QueryTiltPosition(Ser)
                QueryTime = time.time()

                if ElDeg is None:
                    print("EL position unavailable")
                    time.sleep(0.02)
                    continue

                ScanRateDegPerSec = None

                if PreviousElDeg is not None and PreviousTime is not None:
                    DeltaDeg = ElDeg - PreviousElDeg
                    DeltaTime = QueryTime - PreviousTime

                    if DeltaTime > 0:
                        ScanRateDegPerSec = DeltaDeg / DeltaTime
                        print(
                            f"EL={ElDeg:.2f} deg, "
                            f"dEL={DeltaDeg:.2f} deg, "
                            f"dt={DeltaTime:.3f} s, "
                            f"rate={ScanRateDegPerSec:.2f} deg/s, "
                            f"cmd={Direction}"
                        )

                else:
                    print(f"EL={ElDeg:.2f} deg, waiting for next sample")

                # Reverse before configured software limits.
                if Direction == "up" and ElDeg >= (UpperElLimitDeg - LimitMarginDeg):
                    print("Approaching upper EL software limit - reversing down")
                    Stop(Ser)
                    time.sleep(0.2)
                    Direction = "down"
                    CommandDirection(Ser, Direction)
                    StallCount = 0
                    IgnoreStallUntilTime = time.time() + 1.0

                elif Direction == "down" and ElDeg <= (LowerElLimitDeg + LimitMarginDeg):
                    print("Approaching lower EL software limit - reversing up")
                    Stop(Ser)
                    time.sleep(0.2)
                    Direction = "up"
                    CommandDirection(Ser, Direction)
                    StallCount = 0
                    IgnoreStallUntilTime = time.time() + 1.0

                # Backup protection: detect stall / limit switch.
                elif ScanRateDegPerSec is not None and time.time() > IgnoreStallUntilTime:
                    if abs(ScanRateDegPerSec) < MinMovingRateDegPerSec:
                        StallCount += 1
                        print(f"Low EL rate / possible stall sample {StallCount}/{StallSamplesRequired}")
                    else:
                        StallCount = 0

                    if StallCount >= StallSamplesRequired:
                        print("EL LIMIT OR STALL DETECTED - reversing direction")
                        Stop(Ser)
                        time.sleep(0.3)

                        Direction = ReverseDirection(Direction)
                        CommandDirection(Ser, Direction)

                        StallCount = 0
                        IgnoreStallUntilTime = time.time() + 1.0

                PreviousElDeg = ElDeg
                PreviousTime = QueryTime

            time.sleep(0.02)

    finally:
        print("Stop")
        Stop(Ser)

    print("Done")
