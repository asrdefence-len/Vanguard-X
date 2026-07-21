# Vanguard X Stage 4B: scheduler-safe X6-60 telemetry

Stage 4B begins by connecting the real X6-60 telemetry to
`VanguardxMain_scheduler.py` without enabling any motion command.

## Current safety state

- Stage 4A read requests are reused through `X660CanProtocol.py`.
- No slew, position, nudge, brake, configuration, shutdown, or zero command is
  implemented by `X660ReadOnlyController.py`.
- `Stop()` sends no CAN frame.
- The scheduler's automatic startup-pose command is skipped whenever the
  active controller reports `MotionCommandsEnabled = False`.
- The measured North reference is configured as raw angle `-361.53 deg`.
- `X660DirectionSign` remains `0` until a controlled calibration establishes
  whether increasing raw angle is clockwise (`+1`) or anticlockwise (`-1`).
- With direction uncalibrated, pointing state is invalid and radar search
  dwells are held off.

## Files

- `X660ReadOnlyController.py`: SocketCAN telemetry adapter for PointingManager.
- `TestX660ReadOnlyController.py`: mapping and motion-lock regression tests.
- Updated `PTZController.py`: adds `PTZMode = "x660-read-only"`.
- Updated `VanguardxMain_scheduler.py`: skips startup movement for a
  telemetry-only controller and includes the Stage 4B CAN configuration.

The existing Stage 4A `X660CanProtocol.py` must remain in the project folder.

## Safe baseline scheduler launch

Leave this setting unchanged:

```python
"PTZMode": "sim",
```

Then run the scheduler without an RF-output profile:

```bash
python3 -m py_compile VanguardxMain_scheduler.py
python3 -u VanguardxMain_scheduler.py
```

This launches with the simulated X6-60, GUI initially stopped, Ettus
`RECEIVE_ONLY`, timed transmit disabled, and ATR disabled.

Do not use the existing `vanguard.sh` for this baseline.  That script selects
the guarded Stage 3F RF-output profile.

## Real X6-60 telemetry-only launch

After confirming `can0` is already UP at 1 Mbit/s, change only:

```python
"PTZMode": "x660-read-only",
```

Keep:

```python
"X660DirectionSign": 0,
```

Run:

```bash
python3 -m unittest -v TestX660ReadOnlyController.py
python3 -u VanguardxMain_scheduler.py
```

Expected startup includes:

```text
X6-60 SocketCAN telemetry opened on can0, node 1 (UNCALIBRATED; motion locked)
X6-60 startup pose skipped: active controller is telemetry-only
```

Selecting SCAN or issuing a nudge will be refused locally.  No X6-60 motion
CAN frame is constructed or transmitted.

## Next gate

The next Stage 4B action is a separate, explicitly armed calibration harness
for one small speed-limited movement.  Its result will establish
`X660DirectionSign` and verify the physical azimuth limits before any live
movement capability is admitted into the scheduler.
