import serial
import time

Port = "/dev/ttyACM0"
BaudRate = 2400
Address = 1

PanSpeed = 0x5F      # Pelco-D normal max pan speed
TiltSpeed = 0x3F     # Pelco-D normal max tilt speed

def SendPelcoD(Ser, Command1, Command2, Data1, Data2):
    Checksum = (Address + Command1 + Command2 + Data1 + Data2) & 0xFF
    Packet = bytes([0xFF, Address, Command1, Command2, Data1, Data2, Checksum])
    Ser.write(Packet)
    Ser.flush()
    print("TX:", Packet.hex(" "))

    time.sleep(0.1)
    Reply = Ser.read(64)
    if Reply:
        print("RX:", Reply.hex(" "))
    else:
        print("RX: no reply")

with serial.Serial(
    Port,
    BaudRate,
    bytesize=8,
    parity="N",
    stopbits=1,
    timeout=0.5
) as Ser:

    print("Opened", Ser.name)
    print("Baud:", BaudRate)

    print("Pan right fast")
    SendPelcoD(Ser, 0x00, 0x02, PanSpeed, 0x00)
    time.sleep(2.0)

    print("Stop")
    SendPelcoD(Ser, 0x00, 0x00, 0x00, 0x00)
    time.sleep(1.0)

    print("Pan left fast")
    SendPelcoD(Ser, 0x00, 0x04, PanSpeed, 0x00)
    time.sleep(2.0)

    print("Stop")
    SendPelcoD(Ser, 0x00, 0x00, 0x00, 0x00)
    time.sleep(1.0)

    print("Tilt up fast")
    SendPelcoD(Ser, 0x00, 0x08, 0x00, TiltSpeed)
    time.sleep(2.0)

    print("Tilt up fast")
    SendPelcoD(Ser, 0x00, 0x08, 0x00, TiltSpeed)
    time.sleep(2.0)

    print("Stop")
    SendPelcoD(Ser, 0x00, 0x00, 0x00, 0x00)

    print("Done")