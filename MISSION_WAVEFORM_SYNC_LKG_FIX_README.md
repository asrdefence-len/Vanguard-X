# Vanguard X Mission waveform synchronization and last-known-good fix

## Corrected behaviour

- The Mission model is reconciled with the waveform identifiers actually
  advertised by the running `WaveformLibrary`.
- A stale `Golay24_20MHz`, `Golay64_20MHz`, or any other unavailable waveform
  can no longer remain hidden behind a dropdown that displays
  `Frank10_20MHz`.
- When Frank is available, it is the preferred replacement for an unavailable
  stored waveform. Otherwise, the first waveform advertised by the library is
  used.
- New 360 and Sector tasks are created with an available waveform.
- A manually opened Mission file is reconciled with the current waveform
  library before it is shown.
- Every successfully validated Mission draft is saved atomically as the
  last-known-good Mission.
- On the next application start, that last-known-good Mission is restored only
  if it still validates against the current system limits.
- Restoring a Mission never loads it for execution or starts the X6-60. The
  operator must still select Validate Mission, Load Mission, and Start Mission.

## Install

```bash
cd ~/Projects/Software

unzip -o ~/Downloads/VanguardX_Mission_Waveform_Sync_LKG_Fix.zip \
    -d ~/Projects/Software
```

## Test

```bash
cd ~/Projects/Software

python3 -m unittest -v TestMissionModelValidator.py
python3 -m unittest discover -s . -p 'Test*.py' -v
```

## Operator verification

Restart the Vanguard X application and open the Mission page. Task 2 should
show `Frank10_20MHz` both in its editor and in the task-table summary when
Frank is the only advertised waveform.

Select Validate Mission. The previous error:

```text
PrimaryTasks[1].WaveformId: Waveform 'Golay24_20MHz' is not available.
```

or:

```text
PrimaryTasks[1].WaveformId: Waveform 'Golay64_20MHz' is not available.
```

must no longer appear.

After a successful validation, close and restart the application. The same
Mission should reopen as a draft. It must still require explicit Validate,
Load, and Start actions before execution.
