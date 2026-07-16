import serial
import time

Port = "/dev/ttyACM0"
BaudRate = 9600
Address = 1

PanSpeed = 0x5F
TiltSpeed = 0x3F

LeftLimitDeg = 0.0
RightLimitDeg = 306.14

LimitMarginDeg = 1.0

DurationSec = 60.0
QueryIntervalSec = 0.25

MinMovingRateDegPerSec = 1.0
StallSamplesRequired = 4


def SendPelcoD(Ser, Command1, Command2, Data1, Data2, ReadReply=True):
    Checksum = (Address + Command1 + Command2 + Data1 + Data2) & 0xFF
    Packet = bytes([0xFF, Address, Command1, Command2, Data1, Data2, Checksum])

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


def AngleDeltaDeg(CurrentDeg, PreviousDeg):
    Delta = CurrentDeg - PreviousDeg

    while Delta > 180.0:
        Delta -= 360.0

    while Delta < -180.0:
        Delta += 360.0

    return Delta


def QueryPanPosition(Ser):
    Reply = SendPelcoD(Ser, 0x00, 0x51, 0x00, 0x00, ReadReply=True)

    if len(Reply) >= 7:
        if Reply[0] == 0xFF and Reply[1] == Address and Reply[3] == 0x59:
            Raw = (Reply[4] << 8) | Reply[5]
            PanDeg = Raw / 100.0
            return PanDeg

    return None


def PanRight(Ser):
    SendPelcoD(Ser, 0x00, 0x02, PanSpeed, 0x00, ReadReply=False)


def PanLeft(Ser):
    SendPelcoD(Ser, 0x00, 0x04, PanSpeed, 0x00, ReadReply=False)


def Stop(Ser):
    SendPelcoD(Ser, 0x00, 0x00, 0x00, 0x00, ReadReply=False)


with serial.Serial(
    Port,
    BaudRate,
    bytesize=8,
    parity="N",
    stopbits=1,
    timeout=0.2
) as Ser:

    print("Opened", Ser.name)
    print("Baud:", BaudRate)
    print(f"Scanning from {LeftLimitDeg:.2f} deg to {RightLimitDeg:.2f} deg")

    Direction = "right"
    PreviousPanDeg = None
    PreviousTime = None
    StallCount = 0

    StartTime = time.time()
    LastQueryTime = 0.0
    IgnoreStallUntilTime = time.time() + 1.0

    print("Start scan right")
    PanRight(Ser)

    try:
        while time.time() - StartTime < DurationSec:
            Now = time.time()

            if Now - LastQueryTime >= QueryIntervalSec:
                LastQueryTime = Now

                PanDeg = QueryPanPosition(Ser)
                QueryTime = time.time()

                if PanDeg is None:
                    print("PAN position unavailable")
                    time.sleep(0.02)
                    continue

                ScanRateDegPerSec = None

                if PreviousPanDeg is not None and PreviousTime is not None:
                    DeltaDeg = AngleDeltaDeg(PanDeg, PreviousPanDeg)
                    DeltaTime = QueryTime - PreviousTime

                    if DeltaTime > 0:
                        ScanRateDegPerSec = DeltaDeg / DeltaTime
                        print(
                            f"PAN={PanDeg:.2f} deg, "
                            f"dAZ={DeltaDeg:.2f} deg, "
                            f"dt={DeltaTime:.3f} s, "
                            f"rate={ScanRateDegPerSec:.2f} deg/s, "
                            f"dir={Direction}"
                        )

                else:
                    print(f"PAN={PanDeg:.2f} deg, waiting for next sample")

                # Reverse before right mechanical limit.
                if Direction == "right" and PanDeg >= (RightLimitDeg - LimitMarginDeg):
                    print("Approaching right limit - reversing left")
                    Stop(Ser)
                    time.sleep(0.2)
                    PanLeft(Ser)
                    Direction = "left"
                    StallCount = 0
                    IgnoreStallUntilTime = time.time() + 1.0

                # Reverse before left mechanical limit.
                elif Direction == "left" and PanDeg <= (LeftLimitDeg + LimitMarginDeg):
                    print("Approaching left limit - reversing right")
                    Stop(Ser)
                    time.sleep(0.2)
                    PanRight(Ser)
                    Direction = "right"
                    StallCount = 0
                    IgnoreStallUntilTime = time.time() + 1.0

                # Backup protection: detect stall / limit switch.
                elif ScanRateDegPerSec is not None and time.time() > IgnoreStallUntilTime:
                    if abs(ScanRateDegPerSec) < MinMovingRateDegPerSec:
                        StallCount += 1
                    else:
                        StallCount = 0

                    if StallCount >= StallSamplesRequired:
                        print("LIMIT OR STALL DETECTED - reversing direction")
                        Stop(Ser)
                        time.sleep(0.3)

                        if Direction == "right":
                            PanLeft(Ser)
                            Direction = "left"
                        else:
                            PanRight(Ser)
                            Direction = "right"

                        StallCount = 0
                        IgnoreStallUntilTime = time.time() + 1.0

                PreviousPanDeg = PanDeg
                PreviousTime = QueryTime

            time.sleep(0.02)

    finally:
        print("Stop")
        Stop(Ser)

    print("Done")