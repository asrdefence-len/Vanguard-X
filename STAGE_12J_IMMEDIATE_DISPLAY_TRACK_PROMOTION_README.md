# Vanguard X Stage 12J — Immediate Display-Track Promotion

Stage 12J makes an operator-directed initiating-track check take effect
immediately on both tracker representations.

## Fault corrected

Vanguard X currently runs two tracker streams:

- the legacy range/bearing tracker, which is authoritative for tasking; and
- the Earth-referenced ENU tracker, which may be selected for display.

Stage 12I correctly resolved an orange Earth display selection to a tentative
legacy tasking track. However, when the three-pass nod completed, only the
legacy track received the directed result. The Earth tracker was deliberately
frozen during the TRACK task and therefore remained orange until a later
ordinary search-pass boundary.

This could make a successful nod appear not to have confirmed the selected
track, even when the radar console reported that the authoritative track had
promoted.

## Corrected behaviour

- The exact selected display source and track ID are retained in the finite
  TRACK task metadata.
- The legacy tracker remains the sole authority for accepting the gated nod
  hit and deciding promotion or deletion.
- If the operator selected an Earth track, that exact Earth track receives the
  same one-opportunity result before display products are selected.
- A successful `1/3` initiation check therefore becomes confirmed and changes
  from orange to white in the same completion dwell.
- The three physical nod passes still count as one independent tracker
  opportunity.
- Incomplete coverage changes neither track's initiation count.
- If the selected Earth track has genuinely disappeared before completion,
  the console explicitly reports that condition instead of silently leaving
  an orange symbol.

## Expected operator result

After selecting an orange `1/3` track and pressing **Check initiating track**:

```text
Track confirmation: T<n> initiation check queued (+/-5 deg, 3 nod passes)
Track confirmation pointing: T<n> TRACK_SLEW_TO_GATE pass 0/3
Track confirmation pointing: T<n> TRACK_NOD_SCAN pass 1/3
Track confirmation pointing: T<n> TRACK_NOD_SCAN pass 2/3
Track confirmation pointing: T<n> TRACK_NOD_SCAN pass 3/3
Track confirmation: T<n> initiation confirmed (2 hits in 2 opportunities)
```

The selected track should turn white immediately after the final line. It does
not wait for the surrounding 360-degree or sector scan to finish.

## Validation

- 38 focused track-confirmation and Earth-tracker tests pass.
- 459 GUI-free regression tests pass with two expected skips.
- The four new regressions prove exact Earth selection retention, immediate
  Earth promotion, one-opportunity counting, and no ageing on incomplete
  coverage.

`TestRadarDisplayGeometry.py` remains excluded because the existing headless
PyQt stub has no `QWidget`. The unrelated legacy
`Utilities/TestPanTiltPelcoD.py` utility is also outside the operational suite
because its optional `pyserial` dependency is not installed in the validation
environment.
