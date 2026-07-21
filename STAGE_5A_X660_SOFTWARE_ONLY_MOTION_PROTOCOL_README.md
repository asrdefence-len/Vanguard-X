# Stage 5A - X6-60 software-only motion protocol

Stage 5A adds pure eight-byte payload encoders for the documented X6-60
motion commands and verifies them against the examples in Motor Motion
Protocol V4.2.

Implemented payloads:

- `0x81` closed-loop zero-speed stop; the motor remains active and may keep
  applying holding current
- `0xA2` signed speed closed-loop control, 0.01 deg/s per LSB
- `0xA4` absolute multi-turn position, 0.01 deg per LSB, with a uint16
  output-shaft speed limit in deg/s
- `0xA8` incremental multi-turn position, 0.01 deg per LSB, with the same
  speed-limit encoding

Safety boundary:

- `X660MotionProtocol.py` has no CAN, serial, socket, subprocess, bus, open,
  send, write, brake-release, or transmit path.
- `X660ReadOnlyController.MotionCommandsEnabled` remains `False`.
- `CreateX660Controller` still accepts only the simulator and telemetry-only
  real controller modes.
- Position speed limits of zero are rejected because the manual says zero can
  remove the position-loop speed limit in direct tracking mode.
- `0x81` is not a motor-output shutdown. The successfully tested `0x80`
  shutdown utility remains separate and is not connected to the scheduler or
  this payload module.
- Values finer than the documented 0.01-unit resolution are rejected rather
  than silently rounded.
- No brake-release (`0x77`), shutdown (`0x80`), torque (`0xA1`), reset,
  calibration, or configuration command is implemented.

## Apply

From `~/Projects/Software` on branch `x6-60-stage5-motion-control`:

```bash
git status --short
git apply --check Stage5A_X660_Software_Only_Motion_Protocol.patch
git apply Stage5A_X660_Software_Only_Motion_Protocol.patch
```

Do not use `--reject` or `--3way`. If the check fails, stop and inspect the
working tree before continuing.

## Verify

```bash
python3 -m py_compile X660MotionProtocol.py TestX660MotionProtocol.py

python3 -m unittest -v TestX660MotionProtocol.py

python3 -m unittest -v \
    TestX660ReadOnlySafety.py \
    TestX660ReadOnlyController.py \
    TestX660DirectionCalibrationSafety.py \
    TestX660VisibleDirectionCalibrationSafety.py \
    TestX660UnlimitedAzimuth.py \
    TestX660MotionProtocol.py

python3 TestPointingManager.py
python3 TestRadarExecutor.py
python3 TestSchedulerIntegration.py

git diff --check
git status --short
```

All tests are software-only. Do not run the real scheduler and do not connect
this payload module to SocketCAN during Stage 5A.

## Commit scope

Stage these three files only after all verification passes:

```bash
git add -- \
    X660MotionProtocol.py \
    TestX660MotionProtocol.py \
    STAGE_5A_X660_SOFTWARE_ONLY_MOTION_PROTOCOL_README.md
```

Do not stage the delivery patch itself.
