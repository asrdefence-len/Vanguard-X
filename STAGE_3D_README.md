# Stage 3D — guarded combined TX/RX cable loopback

Connect `TX/RX -> at least 30 dB attenuation -> RX2`. Keep the TRM and PA
disconnected or inhibited. ATR GPIO remains disabled. TX gain is selectable
for diagnosis and its actual UHD-coerced value is printed and checked.

The operational radar transmits at T and starts RX at T+6 us, so a direct cable
loopback would finish before collection. This diagnostic therefore starts RX at
T+6 us and transmits at T+10 us. The expected waveform lag is 160 samples at
40 MS/s. This test-only TX offset must not be copied into operational timing.

The program maintains a bounded set of matched pairs and arms each finite RX
window before sending its finite timed TX burst. Queue replenishment happens
immediately after reception; TX-event reporting and matched filtering cannot
consume the scheduling horizon. Processing is deferred until the dwell ends.

```bash
python3 -m unittest -v TestEttusCombinedLoopbackHardwareTest.py

python3 RunEttusCombinedLoopbackHardwareTest.py \
    --serial 34A0320 \
    --prf-hz 4000 \
    --pulses 128 \
    --queue-depth 8 \
    --timeout-sec 0.1 \
    --attenuation-db 70 \
    --i-understand-rf-output-is-enabled \
    --i-confirm-txrx-to-rx2-loopback
```

The hardware program always saves `stage3d_loopback_capture.npz`, including
failed signal-lock runs. Plot it after the complete CPI has been captured:

```bash
python3 PlotStage3DLoopbackCapture.py --show
```

This also writes `stage3d_loopback_capture.png` for sharing and inspection.

If the normal five-microsecond pulse is not visible, use ten repetitions for
a diagnostic 50 us burst. This is RF-path diagnosis, not radar timing:

```bash
python3 RunEttusCombinedLoopbackHardwareTest.py \
    --serial 34A0320 --prf-hz 4000 --pulses 128 --queue-depth 8 \
    --timeout-sec 0.1 --rx-gain-db 30 --attenuation-db 40 \
    --tx-repetitions 10 \
    --i-understand-rf-output-is-enabled \
    --i-confirm-txrx-to-rx2-loopback
```

To reproduce the S-band pulse-visibility structure, use 1 kHz, one outstanding
PRI, TX-first command ordering, RX at T, and diagnostic TX at T+10 us:

```bash
python3 RunEttusCombinedLoopbackHardwareTest.py \
    --serial 34A0320 --prf-hz 1000 --pulses 32 --queue-depth 1 \
    --pair-order tx-first --rx-start-us 0 --tx-test-offset-us 10 \
    --tx-repetitions 10 --timeout-sec 0.5 \
    --rx-gain-db 30 --attenuation-db 40 \
    --i-understand-rf-output-is-enabled \
    --i-confirm-txrx-to-rx2-loopback
```

This deliberately overlaps TX and RX for cable-loopback visibility. ATR, TRM
and PA must remain disconnected/disabled. It is not operational radar timing.

The July 2026 B200mini cable calibration measured the first Frank response 166
samples (4.15 us at 40 MS/s) after the ideal scheduled lag. Apply it explicitly
for the subsequent single-pulse diagnostic with
`--expected-hardware-delay-samples 166`.
