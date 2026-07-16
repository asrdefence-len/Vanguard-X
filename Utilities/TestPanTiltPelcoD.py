import serial
import time

Port = "/dev/ttyACM0"
BaudRate = 1200
Address = 1

def pelco_checksum(address, command1, command2, data1, data2):
    return (address + command1 + command2 + data1 + data2) & 0xFF

def send_pelco_d(ser, command1, command2, data1, data2):
    checksum = pelco_checksum(Address, command1, command2, data1, data2)
    packet = bytes([0xFF, Address, command1, command2, data1, data2, checksum])
    ser.write(packet)
    ser.flush()
    print("Sent:", packet.hex(" "))

with serial.Serial(Port, BaudRate, bytesize=8, parity="N", stopbits=1, timeout=0.5) as ser:
    print("Opened", ser.name)

    # Pan right, speed 0x20
    send_pelco_d(ser, 0x00, 0x02, 0x20, 0x00)
    time.sleep(1.0)

    # Stop
    send_pelco_d(ser, 0x00, 0x00, 0x00, 0x00)
    time.sleep(0.2)

    print("Done")
