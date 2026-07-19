"""Explicit command-line safety profiles for operational Ettus testing."""

from __future__ import annotations

import argparse


def ParseOperatingProfileArguments(argv=None):
    parser = argparse.ArgumentParser(
        description="Run the Vanguard X radar application",
    )
    parser.add_argument(
        "--stage3e1-loopback",
        action="store_true",
        help="enable the guarded, attenuated timed-TX/RX loopback profile",
    )
    parser.add_argument("--attenuation-db", type=float, default=None)
    parser.add_argument("--tx-gain-db", type=float, default=0.0)
    parser.add_argument("--rx-gain-db", type=float, default=10.0)
    parser.add_argument("--maximum-timed-dwells", type=int, default=10)
    parser.add_argument(
        "--i-understand-rf-output-is-enabled",
        action="store_true",
    )
    parser.add_argument(
        "--i-confirm-txrx-to-rx2-loopback",
        action="store_true",
    )
    parser.add_argument(
        "--i-confirm-atr-trm-pa-disabled",
        action="store_true",
    )
    return parser.parse_args(argv)


def ApplyOperatingProfile(config, arguments):
    """Apply a fail-closed operating profile and return its identifier."""

    safety_option_used = bool(
        arguments.attenuation_db is not None
        or arguments.tx_gain_db != 0.0
        or arguments.rx_gain_db != 10.0
        or arguments.i_understand_rf_output_is_enabled
        or arguments.i_confirm_txrx_to_rx2_loopback
        or arguments.i_confirm_atr_trm_pa_disabled
    )

    if not arguments.stage3e1_loopback:
        if safety_option_used:
            raise ValueError(
                "Loopback safety options require --stage3e1-loopback"
            )
        if (
            str(config.get("EttusOperatingMode", "")).upper()
            != "RECEIVE_ONLY"
            or bool(config.get("EttusTimedTransmitEnabled", False))
        ):
            raise RuntimeError(
                "Normal startup must remain fail-closed RECEIVE_ONLY"
            )
        config["Stage3E1LoopbackActive"] = False
        return "RECEIVE_ONLY"

    if not arguments.i_understand_rf_output_is_enabled:
        raise ValueError(
            "Missing --i-understand-rf-output-is-enabled"
        )
    if not arguments.i_confirm_txrx_to_rx2_loopback:
        raise ValueError(
            "Missing --i-confirm-txrx-to-rx2-loopback"
        )
    if not arguments.i_confirm_atr_trm_pa_disabled:
        raise ValueError("Missing --i-confirm-atr-trm-pa-disabled")
    if arguments.attenuation_db is None:
        raise ValueError("--attenuation-db is required for loopback")
    if float(arguments.attenuation_db) < 30.0:
        raise ValueError("Stage 3E1 requires at least 30 dB attenuation")
    if not 0.0 <= float(arguments.tx_gain_db) <= 50.0:
        raise ValueError("Stage 3E1 TX gain must be between 0 and 50 dB")
    if not 0.0 <= float(arguments.rx_gain_db) <= 50.0:
        raise ValueError("Stage 3E1 RX gain must be between 0 and 50 dB")
    if not 1 <= int(arguments.maximum_timed_dwells) <= 100:
        raise ValueError("Maximum timed dwells must be between 1 and 100")

    rx_frequency_hz = float(config.get("EttusRxFrequencyHz", 1.0e9))
    config.update({
        "RadarSource": "ETTUS",
        "EttusOperatingMode": "TIMED_TX_RX",
        "EttusTimedTransmitEnabled": True,
        "EttusRfOutputAcknowledged": True,
        "EttusLoopbackConfirmed": True,
        "EttusExternalAttenuationDb": float(arguments.attenuation_db),
        "EttusTxFrequencyHz": rx_frequency_hz,
        "EttusTxGainDb": float(arguments.tx_gain_db),
        "EttusRxGainDb": float(arguments.rx_gain_db),
        "EttusTxAntenna": "TX/RX",
        "EttusTxChannel": int(config.get("EttusRxChannel", 0)),
        "EttusAtrGpioEnabled": False,
        "Stage3E1LoopbackActive": True,
        "Stage3E1MaximumTimedDwells": int(
            arguments.maximum_timed_dwells
        ),
    })
    return "STAGE3E1_LOOPBACK"
