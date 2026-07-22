# Stage 5D — operational X6-60 controller integration

Stage 5D connects the already-proven X6-60 CAN motion transport to the
Vanguard X `PointingManager` interface. It does not change the established
coordinate calibration:

- North is `0 deg`;
- positive azimuth and positive scan rate are clockwise;
- the azimuth axis is unlimited and wraps naturally through North;
- raw multi-turn encoder position is retained for CAN position commands.

The operational controller implements the same calls already used by the
scheduler:

- `CommandSlew()` for continuous search scanning;
- `SetPanPositionNative()` for track and startup pointing;
- `NudgePanPositionNative()` for operator increments;
- `Stop()` for closed-loop hold;
- `Update()` for measured encoder azimuth, rate, current and error telemetry.

`PointingManager` remains the only owner of search, track, nudge and stop
intent. The new controller only translates that intent to the proven `0xA2`,
`0xA4`, `0xA8`, and `0x81` CAN commands.

## Operational boundary

The committed default remains `X660Mode = "x660-read-only"`. Real motion is
selected explicitly for one application session:

```bash
python3 VanguardxMain_scheduler.py \
    --x660-operational \
    --i-understand-x660-motion-will-occur \
    --i-confirm-x660-motion-area-is-clear
```

The X6-60 options are independent of the Ettus operating profile. With no RF
profile selected, the radar remains receive-only while the real motor and
encoder are operational. The X6-60 options can also be combined with an
explicit guarded Ettus profile when that hardware stage is intended.

The normal Vanguard rate is limited to `14 deg/s`. Position commands use the
nearest multi-turn target, manual nudges are limited to `10 deg` per command,
and any CAN motion failure latches further motion until the controller is
closed and reopened. A stop is attempted when a motion transaction or reported
motor fault fails. Normal close sends closed-loop stop and then confirms motor
output shutdown.

The Vanguard X installation uses the X6-60 as an azimuth-only axis. Startup
initialisation now waits only for azimuth when the controller reports
`SupportsElevation = False`.

## Software-only verification

```bash
python3 -m py_compile \
    X660OperationalController.py \
    X660Controller.py \
    EttusOperatingProfiles.py \
    VanguardxMain_scheduler.py \
    TestX660OperationalController.py

python3 -m unittest -v \
    TestX660ReadOnlySafety.py \
    TestX660ReadOnlyController.py \
    TestX660UnlimitedAzimuth.py \
    TestX660MotionProtocol.py \
    TestX660GuardedMotionTransport.py \
    TestRunX660GuardedMotionTest.py \
    TestX660OperationalController.py
```

All Stage 5D operational-controller tests use injected in-memory telemetry,
transport and shutdown boundaries. They do not inspect or open `can0` and do
not transmit a CAN frame.
