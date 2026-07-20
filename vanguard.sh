#!/usr/bin/env bash
set -euo pipefail

cd "$HOME/Projects/Software"

echo "Vanguard X Stage 3I simulation-only loopback"
echo "Required: TX/RX -> 30 dB attenuation -> RX2"
echo "Required: TRM and PA physically disconnected"
echo "Defaults: TX gain 50 dB, RX gain 30 dB"
echo "WARNING: ATR TX/RX overlap is deliberately enabled."
echo

read -r -p "Type LOOPBACK to confirm the protected setup: " confirmation

if [[ "$confirmation" != "LOOPBACK" ]]; then
    echo "Aborted."
    exit 1
fi

exec python3 VanguardxMain_scheduler.py \
    --stage3i-atr-rf-target-overlap \
    --attenuation-db 30 \
    --loopback-hardware-delay-samples 166 \
    --i-understand-rf-output-is-enabled \
    --i-confirm-txrx-to-rx2-loopback \
    --i-confirm-trm-pa-disconnected \
    --i-confirm-atr-cro-verified
