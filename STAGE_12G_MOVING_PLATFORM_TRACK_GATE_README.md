# Vanguard X Stage 12G — Moving-Platform Track Gate

Stage 12G corrects the start-gate stall observed after an operator selected a
confirmed track and requested three nod passes.

## Cause

The selected track and its plus/minus 5-degree confirmation gate are expressed
in true bearing. The X6-60 position command is platform-relative.

Stage 12F converted the minus-5-degree start gate once, at task activation.
Normal simulation uses a moving circular-route platform, so heading continued
to change while the X6-60 slewed. On arrival, the antenna was at the old
platform-relative command while `PointingManager` compared it with the newly
converted gate. The error exceeded the 0.25-degree tolerance and the task
remained indefinitely in `TRACK_SLEW_TO_GATE`.

## Correction

- Start-gate acquisition now refreshes the platform-relative X6-60 position
  command whenever heading motion displaces it by a meaningful fraction of the
  pointing tolerance.
- Refresh is angular-displacement limited rather than issued on every 50 Hz
  control tick.
- The existing nod-scan yaw compensation remains authoritative after the start
  gate is acquired.
- The console now reports each confirmation pointing phase and pass transition.

The expected sequence is:

```text
Track confirmation: T1 confirmation queued (+/-5 deg, 3 nod passes)
Track confirmation pointing: T1 TRACK_SLEW_TO_GATE pass 0/3 ...
Track confirmation pointing: T1 TRACK_NOD_SCAN pass 1/3 ...
Track confirmation pointing: T1 TRACK_NOD_SCAN pass 2/3 ...
Track confirmation pointing: T1 TRACK_NOD_SCAN pass 3/3 ...
Track confirmation pointing: T1 TRACK_COMPLETE pass 3/3 ...
```

Mission search scheduling remains paused throughout this sequence. After the
confirmation result is applied, the exact interrupted 360 or sector task
resumes as established in Stage 12F.

## Validation

Focused:

```bash
python3 -m unittest -v \
  TestTrackConfirmation.py \
  TestSchedulerIntegration.py \
  TestX660PointingControlLoop.py
```

Syntax:

```bash
python3 -m py_compile \
  PointingManager.py \
  VanguardxMain_scheduler.py \
  TestTrackConfirmation.py
```

The moving-platform regression uses a 20-degree/second acquisition slew,
5-degree/second nod rate, plus/minus 5-degree gate, three passes, and a changing
platform heading. It completes with valid coverage.

The complete GUI-free regression contains 446 passing tests and two expected
skips. `TestRadarDisplayGeometry.py` remains excluded because the existing
headless PyQt stub has no `QWidget`.
