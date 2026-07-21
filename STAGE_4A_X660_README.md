# Stage 4A - X6-60 read-only CAN verification

This stage introduces a dedicated harness for the X6-60 motor/positioning unit.
It is deliberately separate from Vanguard X main, `PointingManager`, radar
transmission, and the legacy `PTZController` classes.

## Safety boundary

The Stage 4A implementation contains only these protocol reads:

- `0xB2`: system software version date
- `0x92`: multi-turn output-shaft angle
- `0x9A`: temperature, brake-command state, bus voltage, and error flags
- `0x9C`: temperature, torque current, output-shaft speed, and coarse angle

It does not implement motor shutdown/stop, brake control, PID or acceleration
writes, encoder-zero changes, watchdog configuration, or any torque, speed, or
position command. The harness defaults to a dry run.

The supplied `van36.py` established the previous hardware assumptions of
SocketCAN `can0`, 1 Mbit/s, and node ID 1. Unlike that legacy application, this
harness does not invoke `sudo`, configure the host CAN interface, write PID
gains, start radar hardware, or create a GUI.

## Hardware-free verification

```bash
python3 -m unittest -v TestX660ReadOnlySafety.py
python3 RunX660ReadOnlyHardwareTest.py
```

The second command prints the exact read-only plan and exits without opening
CAN.

## Host CAN setup

Inspect the interface before changing it:

```bash
ip -details -statistics link show can0
```

If `can0` has not yet been configured, configure it separately from the test
harness:

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 restart-ms 100
sudo ip link set can0 txqueuelen 1000
sudo ip link set can0 up
ip -details -statistics link show can0
```

Do not proceed unless the X6-60 is mechanically secured, the antenna movement
area is clear, power can be isolated immediately, and the unit is stationary.

## Read-only hardware execution

```bash
python3 RunX660ReadOnlyHardwareTest.py \
    --interface can0 \
    --node-id 1 \
    --samples 10 \
    --execute-read-only \
    --i-confirm-x6-60-is-secured-and-area-clear \
    --i-confirm-no-motion-or-configuration-commands
```

Expected success ends with:

```text
PASS: X6-60 read-only CAN identification and telemetry verified
```

Any timeout, unexpected reply ID, wrong command echo, non-eight-byte response,
reported motor error, or non-stationary speed causes failure.

## Deferred Stage 4B

No movement command should be added until Stage 4A passes on the actual unit
and the physical azimuth limits, positive direction, encoder zero, acceptable
speed, emergency isolation procedure, and antenna cable-wrap constraints have
been recorded.
