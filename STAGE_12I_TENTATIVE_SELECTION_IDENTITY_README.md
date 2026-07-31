# Vanguard X Stage 12I — Tentative Selection Identity

Stage 12I preserves the selected display track's initiation class across the
local and Ethernet UI control boundary.

## Corrected behaviour

- Selecting an orange Earth-referenced track carries
  `TrackConfirmWasTentative=True`.
- Earth-to-legacy tasking resolution considers only tentative legacy tracks
  for an initiating-track check.
- A nearby confirmed legacy track can no longer capture the request merely
  because it has a slightly smaller range/bearing error.
- Existing clients that do not provide the new field retain the Stage 12H
  nearest-track behaviour.

The directed tracker rule is unchanged: a complete three-pass nod is one
independent initiation opportunity. A reacquired `1/3` track becomes `2/2` and
promotes immediately.

## Expected operator result

For an orange initiating track, the radar console must say:

```text
Track confirmation: T<n> initiation check queued (+/-5 deg, 3 nod passes)
...
Track confirmation: T<n> initiation confirmed (2 hits in 2 opportunities)
```

If the first line says only `confirmation queued`, the selected tasking track
was already confirmed and the initiating track was not nominated.

## Validation

- 30 focused track-confirmation, radar-link, and control-state tests pass.
- 455 GUI-free regression tests pass with two expected skips.
- The new regression includes one close confirmed track and one tentative
  track, with the confirmed track geometrically closer to the Earth selection.
