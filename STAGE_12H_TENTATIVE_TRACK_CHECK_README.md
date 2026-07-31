# Vanguard X Stage 12H — Tentative Track Check

Stage 12H permits an operator to select a light-orange initiating track, such
as `T1 1/3`, and deliberately check it with the established track noddy.

## Behaviour

- A tentative track presents **Check initiating track** in the Radar tab.
- A confirmed track continues to present **Confirm selected track**.
- Both use the Stage 12G moving-platform-safe sequence: rapid slew to the
  minus-5-degree gate, then three complete nod passes over plus/minus 5 degrees.
- The TRACK task retains exclusive X6-60 and radar ownership. Mission timing
  and scheduled sectors remain paused until the nod finishes.
- All three physical passes are consolidated into one tracker opportunity.
- A `1/3` tentative track that is reacquired becomes two hits and promotes
  immediately to confirmed.
- A complete miss consumes one initiation opportunity. The ordinary 2-of-3
  rule retains or deletes the tentative track.
- Incomplete pointing coverage consumes no opportunity.
- Tentative misses do not invoke the confirmed-track wider-retry/delete policy.
- The exact interrupted 360-degree or sector task resumes afterward.

The Earth display remains non-authoritative. A selected Earth-projected
tentative track is resolved to the nearest legacy tasking track by range and
true bearing before the nod is queued.

## Validation

Focused:

```bash
python3 -m unittest -v \
  TestTrackConfirmation.py \
  TestSchedulerIntegration.py \
  TestRadarLink.py
```

Syntax:

```bash
python3 -m py_compile \
  TrackConfirmation.py \
  RadarTracker.py \
  RadarDisplayQt5.py \
  VanguardxMain_scheduler.py \
  TestTrackConfirmation.py
```

The complete GUI-free regression contains 453 passing tests and two expected
skips. `TestRadarDisplayGeometry.py` remains excluded because the existing
headless PyQt stub has no `QWidget`.
