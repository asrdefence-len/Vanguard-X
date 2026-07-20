# Vanguard X Stage 3A — Ettus receive-only hardware smoke test

This harness validates one real B200mini receive dwell before Main is changed to
use the Ettus source.

## Physical safety conditions

Before running it:

- keep the PA/transmitter disabled or unpowered;
- keep the TRM transmit command inhibited;
- do not connect a high-power transmit signal to the B200mini RX input;
- leave `--enable-atr-gpio` **off** for this first test.

The program does not create a TX streamer, send waveform samples, or call the
TRM controller. Its receive-only pulse plans explicitly set `TxEnabled=False`.

## Install and run

Extract the package into `~/Projects/Software` and run the non-hardware tests:

```bash
cd ~/Projects/Software
python3 -m unittest -v TestEttusReceiveOnlyHardwareHarness.py
```

Confirm UHD can see the device:

```bash
python3 -c "import uhd; print('UHD Python available')"
uhd_find_devices
```

Then run exactly one baseline dwell:

```bash
python3 RunEttusReceiveOnlyHardwareTest.py --serial 34A0320
```

The default test uses:

- `Frank10_20MHz` metadata;
- 40 MS/s;
- 2 kHz PRF / 500 us PRI;
- 32 receive windows;
- 15 km maximum range;
- 4043 samples per window;
- 50 ms UHD command lead;
- ATR GPIO disabled;
- timed transmit disabled.

Do not run the PRF test matrix after a failure. Preserve and review the complete
terminal output first, especially pulse validity, timeout, overflow, late-command
and first-sample timestamp results.
