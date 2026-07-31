# Vanguard X Stage 12E — Track Confirmation and Mission Slew

Stage 12E extends the verified Stage 12D angular-detection build without
changing Golay processing or per-dwell CFAR.

## Operator-visible changes

1. Select a confirmed track on the Radar PPI.
2. Press **Confirm selected track** beneath the target details.
3. Vanguard X interrupts search, moves rapidly to the first edge of the track
   gate, performs three slow nod passes through the predicted bearing, closes
   the angular fit, updates the nominated track, and resumes the interrupted
   search.

The selected track may be shown from either the legacy or Earth display
tracker. Earth-selected tracks are resolved by range and true bearing back to
the authoritative legacy tasking track before motion is commanded.

## Miss evidence

The default new-Mission policy is `RETRY_WIDER_THEN_DELETE`:

- A return inside the directed association gate updates the nominated track.
- The first complete miss queues one retry with twice the angular half-gate,
  bounded by `MaximumGateDeg`.
- A second fully covered miss deletes the track.
- An incomplete pointing move or timeout is not negative target evidence; the
  track remains and coasts normally.

`COAST`, `DEFER`, `DELETE`, and the earlier retry policies remain selectable in
the Mission editor.

## Duplicate-track suppression

After each completed scan pass, a nearby stale or tentative companion track is
merged into the stronger established track when it is within:

- 200 m in range; and
- 5 degrees in bearing.

Two confirmed tracks independently updated by separate plots in the same pass
are retained, preventing the suppression rule from automatically collapsing
two genuinely close vessels. A successful directed confirmation also merges a
nearby stale companion into the nominated track.

## Mission sector transition

On a primary Mission change from 360 scan to sector scan:

1. The sector task enters `SLEW_TO_START`.
2. The X6-60 uses a precise position command at
   `X660MissionTransitionSlewRateDegPerSec` (default 40 degrees/second).
3. Radar dwells are inhibited during this repositioning movement.
4. On reaching the sector start, the controller changes to the operator's
   sector scan rate and begins surveillance.

This rapid repositioning is separate from sector endpoint reversal and from
the slower track nod rate.

## Validation

Focused:

```bash
python3 -m unittest -v \
  TestTrackConfirmation.py \
  TestAngularDetectionProcessor.py \
  TestRadarLink.py \
  TestRadarTimingIntegration.py
```

Syntax:

```bash
python3 -m py_compile \
  TrackConfirmation.py \
  PointingManager.py \
  RadarTracker.py \
  RadarExecutor.py \
  VanguardxMain_scheduler.py
```

The complete GUI-free regression contains 440 passing tests and two expected
skips. `TestRadarDisplayGeometry.py` remains excluded because the existing
headless PyQt stub has no `QWidget`.
