import serial
import time

Port = "/dev/ttyACM0"
BaudRate = 2400
Address = 1

LeftLimitDeg = 10.0
RightLimitDeg = 300.0


def SendPelcoD(Ser, Command1, Command2, Data1, Data2, ReadReply=True):
    Checksum = (Address + Command1 + Command2 + Data1 + Data2) & 0xFF
    Packet = bytes([0xFF, Address, Command1, Command2, Data1, Data2, Checksum])

    Ser.reset_input_buffer()
    Ser.write(Packet)
    Ser.flush()

    print("TX:", Packet.hex(" "))

    if not ReadReply:
        return b""

    time.sleep(0.1)
    Reply = Ser.read(64)

    if Reply:
        print("RX:", Reply.hex(" "))
    else:
        print("RX: no reply")

    return Reply


def QueryPanPosition(Ser):
    Reply = SendPelcoD(Ser, 0x00, 0x51, 0x00, 0x00, ReadReply=True)

    if len(Reply) < 7:
        return None

    for i in range(len(Reply) - 6):
        Frame = Reply[i:i + 7]

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
        PanDeg = Raw / 100.0

        return PanDeg

    return None


def Stop(Ser):
    SendPelcoD(Ser, 0x00, 0x00, 0x00, 0x00, ReadReply=False)
    time.sleep(0.03)
    SendPelcoD(Ser, 0x00, 0x00, 0x00, 0x00, ReadReply=False)


def ClampPanDeg(PanDeg):
    if PanDeg < LeftLimitDeg:
        return LeftLimitDeg

    if PanDeg > RightLimitDeg:
        return RightLimitDeg

    return PanDeg


def SetPanPosition(Ser, PanDeg):
    PanDeg = ClampPanDeg(PanDeg)

    Raw = int(round(PanDeg * 100.0)) & 0xFFFF
    Data1 = (Raw >> 8) & 0xFF
    Data2 = Raw & 0xFF

    print(f"SET PAN target={PanDeg:.2f} deg raw={Raw}")
    SendPelcoD(Ser, 0x00, 0x4B, Data1, Data2, ReadReply=False)


def NudgePanPosition(Ser, IncrementDeg):
    Current = QueryPanPosition(Ser)

    if Current is None:
        print("Cannot nudge: pan position query failed")
        return

    Target = ClampPanDeg(Current + IncrementDeg)

    print(f"NUDGE current={Current:.2f} deg increment={IncrementDeg:.2f} target={Target:.2f}")
    SetPanPosition(Ser, Target)


def WatchPosition(Ser, DurationSec=3.0):
    Start = time.time()

    while time.time() - Start < DurationSec:
        Pan = QueryPanPosition(Ser)

        if Pan is None:
            print("PAN unavailable")
        else:
            print(f"PAN={Pan:.2f} deg")

        time.sleep(0.25)


with serial.Serial(
    Port,
    BaudRate,
    bytesize=8,
    parity="N",
    stopbits=1,
    timeout=0.3,
) as Ser:

    print("Opened", Ser.name)
    print("Baud:", BaudRate)
    print("Pelco-D native position command test")
    print()

    try:
        while True:
            print()
            print("Menu")
            print("1 = query pan position")
            print("2 = nudge +1 deg")
            print("3 = nudge -1 deg")
            print("4 = nudge +2 deg")
            print("5 = nudge -2 deg")
            print("6 = goto absolute pan position")
            print("7 = stop")
            print("q = quit")
            Cmd = input("PTZ_POS> ").strip().lower()

            if Cmd == "1":
                Pan = QueryPanPosition(Ser)
                print(f"PAN={Pan}")

            elif Cmd == "2":
                NudgePanPosition(Ser, +1.0)
                WatchPosition(Ser, 3.0)

            elif Cmd == "3":
                NudgePanPosition(Ser, -1.0)
                WatchPosition(Ser, 3.0)

            elif Cmd == "4":
                NudgePanPosition(Ser, +2.0)
                WatchPosition(Ser, 3.0)

            elif Cmd == "5":
                NudgePanPosition(Ser, -2.0)
                WatchPosition(Ser, 3.0)

            elif Cmd == "6":
                Target = float(input("Target pan deg: "))
                SetPanPosition(Ser, Target)
                WatchPosition(Ser, 5.0)

            elif Cmd == "7":
                Stop(Ser)

            elif Cmd == "q":
                Stop(Ser)
                break

            else:
                print("Unknown command")

    finally:
        Stop(Ser)
        print("Done")