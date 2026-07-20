# Stage 3C — guarded timed-transmit test

This package tests the B200mini timed-TX queue independently of ATR, the TRM,
the PA, and radar receive processing. It sends one finite timed burst per PRI,
keeps at most 20 bursts outstanding, checks every `send()` sample count, and
requires a UHD burst acknowledgement for every pulse. Any late/time event,
underflow, sequence error, short send, or missing acknowledgement fails.

## Mandatory RF setup

1. Keep the TRM and PA disconnected or inhibited.
2. Keep ATR GPIO disabled.
3. Connect the B200mini TX/RX port through the attenuator to a 50-ohm load.
4. Set the attenuator to 70 dB for the initial test and confirm its RF power
   and frequency ratings are adequate.
5. Do not connect the attenuated output to the B200 RX input in this first test.

The program fixes B200 TX gain at 0 dB and requires both an attenuation value
of at least 30 dB and an explicit RF-output acknowledgement flag.

## Initial test

```bash
python3 -m unittest -v TestEttusTimedTransmitHardwareTest.py

python3 RunEttusTimedTransmitHardwareTest.py \
    --serial 34A0320 \
    --prf-hz 4000 \
    --pulses 128 \
    --attenuation-db 70 \
    --i-understand-rf-output-is-enabled
```

Stop if the RF arrangement differs from the mandatory setup or any UHD TX
event is reported. This package does not modify `EttusRadarSource.py`; combined
TX/RX operation belongs to the following stage after this isolated test passes.
