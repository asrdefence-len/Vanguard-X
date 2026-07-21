# Vanguard X Stage 4E — X6-60 test and utility cleanup

This follow-up completes the X6-60-only migration in the remaining regression
tests and navigation documentation. It does not enable real X6-60 motion.

## Changes

- Replaces `SimulatedPTZController` with `SimulatedX660Controller`.
- Replaces the `ptz=` pointing-manager interface with `x660=`.
- Removes obsolete 10–300 degree simulated position limits.
- Replaces `PTZScanSlewRateDegPerSec` with
  `X660ScanSlewRateDegPerSec` in timing and Golay tests.
- Removes remaining PTZ wording from `NavigationState.py`.
- Removes the two obsolete Pelco utility programs.

The patch contains only changes to the six uploaded Python files. This README
is deliberately not included in the patch.

## Apply

Copy these files into `~/Projects/Software`:

- `Stage4E_X660_Test_Cleanup.patch`
- `STAGE_4E_X660_TEST_CLEANUP_README.md`

Then run:

```bash
cd ~/Projects/Software

git rm --ignore-unmatch -- \
    Utilities/nudgeptz.py \
    Utilities/test_ptz_slew_query_reassert.py

git apply --check Stage4E_X660_Test_Cleanup.patch
git apply Stage4E_X660_Test_Cleanup.patch
```

If `git apply --check` reports an error, stop and do not run the second
`git apply` command.

## Verify

Run each command separately:

```bash
python3 -m py_compile NavigationState.py TestPointingManager.py TestRadarExecutor.py TestGolayOperational.py TestRadarTimingControls.py TestRadarTimingIntegration.py
```

```bash
python3 TestPointingManager.py
```

```bash
python3 TestRadarExecutor.py
```

```bash
python3 -m unittest -v TestX660ReadOnlySafety.py TestX660ReadOnlyController.py TestX660UnlimitedAzimuth.py TestGolayOperational.py TestRadarTimingControls.py TestRadarTimingIntegration.py
```

```bash
python3 TestSchedulerIntegration.py
```

Check formatting and remaining legacy source references:

```bash
git diff --check

git grep -n -I -E 'Pelco|PTZ|Ptz|ptz' -- \
    '*.py' \
    ':!TestX660UnlimitedAzimuth.py'

git status --short
```

The legacy-source grep should produce no output. The excluded
`TestX660UnlimitedAzimuth.py` intentionally contains old names as rejection
test data.

Do not start the real scheduler or commit until the verification output has
been reviewed. Real X6-60 movement remains motion-locked.
