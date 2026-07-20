# Vanguard X Stage 3B — fixed-depth 20 individual-PRI pipeline

This package replaces the adaptive queue-depth implementation with a simpler
constant bound of 20 individual finite PRI pairs.

The hardware has demonstrated that 20 pre-armed finite RX commands are clean at
4 kHz, while a full 128-command dwell queue can hang. The source therefore:

1. performs one transmitter-inhibited finite RX warm-up after setting 40 MS/s;
2. primes up to 20 individual timed TX/RX pairs;
3. collects the oldest finite RX window;
4. appends exactly one future pair, maintaining depth 20;
5. drains naturally during the final 20 pulses of the CPI.

There is no continuous receive and no adaptive scheduling rule. Every RX window
and future TX burst remains finite and pulse-specific, so FPGA ATR can switch
between TX and RX for every PRI.

## Queue horizon

With 20 pulses queued, committed look-ahead is:

| PRF | Queue horizon |
|---:|---:|
| 1 kHz | 20 ms |
| 2 kHz | 10 ms |
| 2.5 kHz | 8 ms |
| 4 kHz | 5 ms |

Radar tasks already execute and process complete CPIs, so this bounded committed
horizon is acceptable. The configured limit is validated as 1-32, but the
operational baseline is fixed at 20 and is not an operator UI control.

Stage 3B leaves all timed-TX hooks disabled and keeps ATR disabled.

## Install and test

Extract into `~/Projects/Software`, replacing the included files, then run:

```bash
cd ~/Projects/Software
python3 -m unittest -v \
    TestEttusBoundedPriPipeline.py \
    TestEttusReceiveOnlyHardwareHarness.py \
    TestRadarTimingIntegration.py
```

Then validate one 4 kHz dwell:

```bash
python3 RunEttusReceiveOnlyHardwareTest.py \
    --serial 34A0320 \
    --prf-hz 4000 \
    --pulses 128
```

Expected structure:

```text
RX execution:   bounded individual PRI pipeline, depth=20
queue horizon:  5.000 ms
UHD RX commands: 128 individual finite windows; depth=20, max outstanding=20
```

Keep ATR and timed TX disabled. Stop after any failure and preserve the complete
output, including any `pulse error` lines.
