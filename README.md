# Minimal continuous CRO sine test

TX only. No RX streamer, pulses, queue, ATR, TRM, PA, or radar processing.
Connect `TX/RX -> 30 dB or greater attenuation -> CRO 50-ohm input`.

```bash
python3 RunEttusCroContinuousSine.py \
  --serial 34A0320 --duration-sec 30 --attenuation-db 30 \
  --i-understand-rf-output-is-enabled \
  --i-confirm-scope-is-50-ohm-terminated
```

Defaults generate approximately 100 MHz for the selected duration: a 99 MHz
RF centre plus a +1 MHz complex baseband sine. TX gain is fixed at 0 dB.
