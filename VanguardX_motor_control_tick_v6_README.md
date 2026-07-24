# Vanguard X v6 — 50 Hz X6-60 pointing control

This drop-in update separates X6-60 endpoint control from the radar dwell
cadence.

- Radar dwell cadence remains 100 ms (10 Hz).
- X6-60 telemetry is refreshed every 20 ms (50 Hz).
- `PointingManager.Update()` evaluates sector braking and reversal every 20 ms.
- The scheduler remains single-threaded, keeping CAN transactions serial.
- Missed control periods are skipped rather than replayed as CAN bursts.
- Actual control intervals, maximum jitter, and missed ticks are recorded.
- Existing speed-dependent braking compensation remains unchanged.
- Manual movement and track-position slew behaviour are unchanged.

Install from `~/Projects/Software`:

```bash
unzip -o ~/Downloads/VanguardX_motor_control_tick_v6_dropin.zip
python3 TestX660PointingControlLoop.py
python3 TestX660ScanBraking.py
python3 TestPointingManager.py
```

Then launch:

```bash
python3 VanguardxMain_scheduler.py --system-hard
```

Repeat the 10°–120° sector scan at 40°/s for at least three reversals and record
the physical lower and upper turning points.
