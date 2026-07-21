# Vanguard X Stage 4D — X6-60-only unlimited azimuth

This update removes the Pelco-D controller and the old PTZ simulator from the
active Vanguard architecture.  `PTZController.py` is deleted and replaced by
`X660Controller.py`.

The remaining software simulator is explicitly an X6-60 simulator.  It models:

- unlimited multi-turn azimuth;
- North as `000.00 deg`;
- increasing Vanguard azimuth clockwise;
- natural display wrapping from `359.99 deg` to `000.00 deg`;
- a preserved continuous `RawAngleDeg` value;
- sector scans that may cross North; and
- optional continuous-clockwise marine-radar scanning.

The real controller remains `X660ReadOnlyController`.  Motion is still locked.
This update does not authorise scheduler movement or transmit RF.

## Apply the patch

Close `VanguardxMain_scheduler.py`, then run:

```bash
cd ~/Projects/Software

git apply --check Stage4D_X660_Only_Unlimited_Azimuth.patch
git apply Stage4D_X660_Only_Unlimited_Azimuth.patch
```

The patch deliberately deletes `PTZController.py` and adds
`X660Controller.py`.

## Software checks

```bash
python3 -m py_compile \
    VanguardxMain_scheduler.py \
    X660Controller.py \
    X660ReadOnlyController.py \
    PointingManager.py \
    RadarTasks.py \
    RadarScheduler.py \
    RadarExecutor.py \
    RadarDisplayQt5.py \
    TestSchedulerIntegration.py \
    TestX660UnlimitedAzimuth.py

python3 -m unittest -v \
    TestX660ReadOnlySafety.py \
    TestX660ReadOnlyController.py \
    TestX660UnlimitedAzimuth.py

python3 TestSchedulerIntegration.py
```

## Safe real-hardware check

Keep the X6-60 configuration as:

```python
"EnableX660": True,
"X660Mode": "x660-read-only",
"X660NorthRawAngleDeg": -361.53,
"X660DirectionSign": +1,
"X660ScanPattern": "SECTOR",
```

Launch the scheduler and leave the GUI in `STOP`:

```bash
python3 -u VanguardxMain_scheduler.py
```

Expected startup includes:

```text
X6-60 SocketCAN telemetry opened on can0, node 1 (calibrated; motion locked)
X6-60 startup pose skipped: active controller is telemetry-only
```

The PPI beam should now refresh from measured encoder telemetry while stopped.
For example, raw `-350.00 deg` with North raw `-361.53 deg` displays as
approximately `011.53 deg`.

Do not press SCAN to test movement.  The read-only controller will continue to
refuse every movement command.

## Search patterns

The default remains a reversing sector search:

```python
"X660ScanPattern": "SECTOR",
```

The architecture also represents continuous clockwise rotation without
reversing at North:

```python
"X660ScanPattern": "CONTINUOUS_CW",
```

`CONTINUOUS_CW` is software-tested but cannot move the real X6-60 until a
separate motion-enabled controller is deliberately implemented and verified.
