# Vanguard X Stage 12F — Exclusive Track-Confirmation Resource

Stage 12F corrects the resource-ownership conflict found during operator
confirmation of a selected track.

## Correct execution order

1. The current 360 or sector search state is retained.
2. The selected confirmed track queues an exclusive TRACK task.
3. Mission active-task time and periodic-sector due time are held.
4. The X6-60 slews rapidly to target bearing minus 5 degrees.
5. The X6-60 completes three full nod passes between minus and plus 5 degrees.
6. Angular plots from all three passes are retained as confirmation evidence.
7. The nominated track is updated, retried, deleted, or allowed to coast
   according to the selected Mission miss policy.
8. The interrupted search task resumes with its retained scan state.
9. Periodic Mission scheduling resumes only after TRACK releases the radar.

A queued or active confirmation therefore cannot be overridden by a sector
transition. The executor also checks physical pointing ownership on every task
step and immediately reacquires the scheduled TRACK task if another path has
replaced it.

## Gate compatibility

Explicit operator confirmation now uses a minimum half-gate of 5 degrees.
Older Mission files containing `GateHalfWidthDeg: 2.0` are automatically
upgraded to plus/minus 5 degrees for the confirmation request. A larger
operator-selected gate is retained, bounded by `MaximumGateDeg`.

The first complete miss under `RETRY_WIDER_THEN_DELETE` queues a wider retry.
With the default limits, a plus/minus 5-degree first attempt becomes a
plus/minus 8-degree retry. A second fully covered miss deletes the track.
Incomplete coverage or a motion timeout does not count as negative evidence.

## Validation

Focused:

```bash
python3 -m unittest -v \
  TestTrackConfirmation.py \
  TestMissionExecution.py \
  TestRadarScheduler.py \
  TestSchedulerIntegration.py
```

Syntax:

```bash
python3 -m py_compile \
  MissionExecutionController.py \
  RadarScheduler.py \
  RadarExecutor.py \
  TrackConfirmation.py \
  MissionProfile.py \
  VanguardxMain_scheduler.py
```

The complete GUI-free regression contains 444 passing tests and two expected
skips. `TestRadarDisplayGeometry.py` remains excluded because the existing
headless PyQt stub has no `QWidget`.

