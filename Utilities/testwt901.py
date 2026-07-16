import serial
import time
import csv

Port = "/dev/ttyUSB0"
BaudRate = 9600
LogFile = "wt901_log.csv"

LatestGyro = None
LastYawDeg = None
YawUnwrappedDeg = None
RadarAzUnwrappedDeg = 0.0


def Signed16(Lo, Hi):
    Value = (Hi << 8) | Lo
    if Value >= 32768:
        Value -= 65536
    return Value


def ChecksumOk(Frame):
    return (sum(Frame[0:10]) & 0xFF) == Frame[10]


def Wrap360(AngleDeg):
    return AngleDeg % 360.0


def AngleDeltaDeg(CurrentDeg, PreviousDeg):
    Delta = CurrentDeg - PreviousDeg

    while Delta > 180.0:
        Delta -= 360.0

    while Delta < -180.0:
        Delta += 360.0

    return Delta


def DecodeFrame(Frame):
    global LatestGyro
    global LastYawDeg
    global YawUnwrappedDeg
    global RadarAzUnwrappedDeg

    if len(Frame) != 11 or Frame[0] != 0x55:
        return None

    if not ChecksumOk(Frame):
        return None

    FrameType = Frame[1]

    Xraw = Signed16(Frame[2], Frame[3])
    Yraw = Signed16(Frame[4], Frame[5])
    Zraw = Signed16(Frame[6], Frame[7])

    Now = time.time()

    if FrameType == 0x52:
        Gx = Xraw / 32768.0 * 2000.0
        Gy = Yraw / 32768.0 * 2000.0
        Gz = Zraw / 32768.0 * 2000.0

        LatestGyro = {
            "gyro_time": Now,
            "gyro_x_deg_s": Gx,
            "gyro_y_deg_s": Gy,
            "gyro_z_deg_s": Gz,
            "gyro_x_raw": Xraw,
            "gyro_y_raw": Yraw,
            "gyro_z_raw": Zraw,
        }

        return None

    if FrameType != 0x53:
        return None

    RollDeg = Xraw / 32768.0 * 180.0
    PitchDeg = Yraw / 32768.0 * 180.0
    YawDeg = Zraw / 32768.0 * 180.0

    if LastYawDeg is None:
        LastYawDeg = YawDeg
        YawUnwrappedDeg = YawDeg
        RadarAzUnwrappedDeg = 0.0
        DeltaYawDeg = 0.0
    else:
        DeltaYawDeg = AngleDeltaDeg(YawDeg, LastYawDeg)
        YawUnwrappedDeg += DeltaYawDeg

        # Your observed convention:
        # CW makes yaw decrease, so radar CW-positive azimuth is -yaw motion.
        RadarAzUnwrappedDeg += -DeltaYawDeg

        LastYawDeg = YawDeg

    Row = {
        "time": Now,
        "frame_hex": Frame.hex(" "),
        "roll_raw": Xraw,
        "pitch_raw": Yraw,
        "yaw_raw": Zraw,
        "roll_deg": RollDeg,
        "pitch_deg": PitchDeg,
        "yaw_deg": YawDeg,
        "delta_yaw_deg": DeltaYawDeg,
        "yaw_unwrapped_deg": YawUnwrappedDeg,
        "radar_az_unwrapped_deg": RadarAzUnwrappedDeg,
        "radar_az_wrapped_deg": Wrap360(RadarAzUnwrappedDeg),
        "radar_el_deg": PitchDeg,
    }

    if LatestGyro is not None:
        Row.update(LatestGyro)
    else:
        Row.update({
            "gyro_time": "",
            "gyro_x_raw": "",
            "gyro_y_raw": "",
            "gyro_z_raw": "",
            "gyro_x_deg_s": "",
            "gyro_y_deg_s": "",
            "gyro_z_deg_s": "",
        })

    return Row


with serial.Serial(
    Port,
    BaudRate,
    bytesize=8,
    parity="N",
    stopbits=1,
    timeout=0.1,
) as Ser, open(LogFile, "w", newline="") as CsvFile:

    FieldNames = [
        "time",
        "frame_hex",
        "roll_raw",
        "pitch_raw",
        "yaw_raw",
        "roll_deg",
        "pitch_deg",
        "yaw_deg",
        "delta_yaw_deg",
        "yaw_unwrapped_deg",
        "radar_az_unwrapped_deg",
        "radar_az_wrapped_deg",
        "radar_el_deg",
        "gyro_time",
        "gyro_x_raw",
        "gyro_y_raw",
        "gyro_z_raw",
        "gyro_x_deg_s",
        "gyro_y_deg_s",
        "gyro_z_deg_s",
    ]

    Writer = csv.DictWriter(CsvFile, fieldnames=FieldNames)
    Writer.writeheader()

    print("Opened WT901 on", Ser.name)
    print("Baud:", BaudRate)
    print("Logging WT901 angle + gyro frames to", LogFile)
    print("Test: rotate CCW 180 deg, then CW 180 deg back to start.")
    print("Press Ctrl+C to stop.")
    print()

    Buffer = bytearray()
    Count = 0

    try:
        while True:
            ByteData = Ser.read(1)

            if not ByteData:
                continue

            Byte = ByteData[0]

            if len(Buffer) == 0:
                if Byte != 0x55:
                    continue

            Buffer.append(Byte)

            if len(Buffer) == 11:
                Row = DecodeFrame(bytes(Buffer))

                if Row is not None:
                    Writer.writerow(Row)
                    CsvFile.flush()
                    Count += 1

                    print(
                        f"\rLogged {Count} angle frames | "
                        f"yaw={Row['yaw_deg']:8.2f} | "
                        f"yaw_unwrap={Row['yaw_unwrapped_deg']:8.2f} | "
                        f"radar_az={Row['radar_az_unwrapped_deg']:8.2f} | "
                        f"gyro_z={Row['gyro_z_deg_s']:8.2f}     ",
                        end="",
                        flush=True,
                    )

                Buffer.clear()

    except KeyboardInterrupt:
        print()
        print("Stopped")
        print("Saved:", LogFile)
        