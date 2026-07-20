# TX-only CRO test

Connect `B200mini TX/RX -> 30 dB or greater attenuator -> oscilloscope`.
Disconnect RX2, the TRM and PA. Use the CRO's native 50-ohm input or a proper
external 50-ohm feed-through terminator. Use coax, not a probe ground lead.

Defaults produce approximately 100 MHz: 99 MHz RF centre plus a +1 MHz complex
baseband tone. There are 100 bursts, each 50 us long, at 100 Hz PRF. TX gain is
fixed at 0 dB. The expected burst period is 10 ms.

```bash
python3 RunEttusCroTransmitTest.py \
  --serial 34A0320 --attenuation-db 30 \
  --i-understand-rf-output-is-enabled \
  --i-confirm-scope-is-50-ohm-terminated
```

Suggested CRO setup: 50 ohm input, at least 500 MS/s sample rate, trigger on a
rising RF/envelope edge, 10 us/div initially, and adjust vertical sensitivity
for the attenuated signal.
