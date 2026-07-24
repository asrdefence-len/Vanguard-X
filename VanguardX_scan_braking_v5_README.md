# Vanguard X scan braking compensation — v5

This update preserves the operator-entered sector limits as the intended
physical X6-60 turning points.  In operational CAN mode, `PointingManager`
commands the speed reversal before the boundary using:

```text
advance_deg = measured_speed_deg_per_sec²
              / (2 × effective_deceleration_deg_per_sec²)
              + measured_speed_deg_per_sec × command_latency_sec
```

Initial characterised settings:

- Effective output-shaft deceleration: `60 deg/s²`
- Telemetry/scheduler/CAN latency allowance: `0.05 s`
- Static endpoint margin: `1.0 deg`
- Braking compensation: enabled for the operational X6-60 only

Expected initial reversal advances:

| Measured speed | Reversal advance |
|---:|---:|
| 20 deg/s | 4.33 deg |
| 40 deg/s | 15.33 deg |
| 60 deg/s | 33.00 deg |

The calculation uses actual X6-60 speed telemetry. During acceleration or
deceleration, the advance therefore follows the current speed rather than the
dashboard selection.

## Install and test

```bash
cd ~/Projects/Software
unzip -o ~/Downloads/VanguardX_scan_braking_v5_dropin.zip

python3 TestX660ScanBraking.py
python3 TestPointingManager.py
python3 TestDashboardScanRate.py
python3 TestRadarLink.py
```

Start the receive-only hardware profile:

```bash
python3 VanguardxMain_scheduler.py --system-hard
```

For the first hardware check:

1. Select the 10–120 degree sector.
2. Select 40 deg/s.
3. Run at least three reversals.
4. Press STOP and allow the motor to settle.
5. Record the lower and upper physical turning points.

Do not tune motor PID values or send manual `cansend` motion frames during this
test.  The position-slew/track-revisit path is unchanged by this update.
