"""Explicit command-line safety profiles for operational Ettus testing."""

from __future__ import annotations

import argparse


DEFAULT_LOOPBACK_TX_GAIN_DB = 0.0
DEFAULT_LOOPBACK_RX_GAIN_DB = 10.0
DEFAULT_STAGE3I_TX_GAIN_DB = 50.0
DEFAULT_STAGE3I_RX_GAIN_DB = 30.0


def ParseOperatingProfileArguments(argv=None):
    parser = argparse.ArgumentParser(
        description="Run the Vanguard X radar application",
    )
    profile = parser.add_mutually_exclusive_group()
    profile.add_argument(
        "--stage3e1-loopback",
        action="store_true",
        help="enable the guarded timed-TX/RX loopback profile",
    )
    profile.add_argument(
        "--stage3f-rf-target",
        action="store_true",
        help="enable the guarded delayed-RF target-emulator profile",
    )
    profile.add_argument(
        "--stage3h-atr-loopback",
        action="store_true",
        help="enable guarded timed-TX/RX loopback with verified ATR GPIO",
    )
    profile.add_argument(
        "--stage3i-atr-rf-target-overlap",
        action="store_true",
        help=(
            "enable simulation-only RF target emulation with deliberate "
            "ATR TX/RX overlap"
        ),
    )

    parser.add_argument("--attenuation-db", type=float, default=None)
    parser.add_argument(
        "--tx-gain-db",
        type=float,
        default=None,
        help=(
            "override TX gain; defaults to 50 dB for Stage 3I and "
            "0 dB for other guarded loopback profiles"
        ),
    )
    parser.add_argument(
        "--rx-gain-db",
        type=float,
        default=None,
        help=(
            "override RX gain; defaults to 30 dB for Stage 3I and "
            "10 dB for other guarded loopback profiles"
        ),
    )
    parser.add_argument("--target-range-km", type=float, default=6.0)
    parser.add_argument("--target-bearing-deg", type=float, default=80.0)
    parser.add_argument(
        "--target-radial-velocity-mps",
        type=float,
        default=0.0,
    )
    parser.add_argument(
        "--loopback-hardware-delay-samples",
        type=int,
        default=166,
    )

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
    parser.add_argument(
        "--i-confirm-trm-pa-disconnected",
        action="store_true",
    )
    parser.add_argument(
        "--i-confirm-atr-cro-verified",
        action="store_true",
    )
    return parser.parse_args(argv)


def _ValidateCommonLoopbackArguments(arguments, *, atr_enabled=False):
    if not arguments.i_understand_rf_output_is_enabled:
        raise ValueError(
            "Missing --i-understand-rf-output-is-enabled"
        )
    if not arguments.i_confirm_txrx_to_rx2_loopback:
        raise ValueError(
            "Missing --i-confirm-txrx-to-rx2-loopback"
        )
    if atr_enabled:
        if not getattr(arguments, "i_confirm_trm_pa_disconnected", False):
            raise ValueError("Missing --i-confirm-trm-pa-disconnected")
        if not getattr(arguments, "i_confirm_atr_cro_verified", False):
            raise ValueError("Missing --i-confirm-atr-cro-verified")
    elif not arguments.i_confirm_atr_trm_pa_disabled:
        raise ValueError("Missing --i-confirm-atr-trm-pa-disabled")
    if arguments.attenuation_db is None:
        raise ValueError("--attenuation-db is required for loopback")
    if float(arguments.attenuation_db) < 30.0:
        raise ValueError("Guarded loopback requires at least 30 dB attenuation")
    if not 0.0 <= float(arguments.tx_gain_db) <= 50.0:
        raise ValueError("Loopback TX gain must be between 0 and 50 dB")
    if not 0.0 <= float(arguments.rx_gain_db) <= 50.0:
        raise ValueError("Loopback RX gain must be between 0 and 50 dB")


def _ApplyCommonTimedLoopback(config, arguments):
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
        "EttusAtrAllowOverlapForSimulation": False,
        "Stage3E1LoopbackActive": True,
    })


def _ApplyProfileGainDefaults(arguments):
    """Resolve omitted gains without changing the safe global baseline."""

    stage3i = bool(getattr(
        arguments,
        "stage3i_atr_rf_target_overlap",
        False,
    ))
    if arguments.tx_gain_db is None:
        arguments.tx_gain_db = (
            DEFAULT_STAGE3I_TX_GAIN_DB
            if stage3i
            else DEFAULT_LOOPBACK_TX_GAIN_DB
        )
    if arguments.rx_gain_db is None:
        arguments.rx_gain_db = (
            DEFAULT_STAGE3I_RX_GAIN_DB
            if stage3i
            else DEFAULT_LOOPBACK_RX_GAIN_DB
        )


def ApplyOperatingProfile(config, arguments):
    """Apply a fail-closed operating profile and return its identifier."""

    profile_selected = bool(
        arguments.stage3e1_loopback
        or arguments.stage3f_rf_target
        or getattr(arguments, "stage3h_atr_loopback", False)
        or getattr(
            arguments,
            "stage3i_atr_rf_target_overlap",
            False,
        )
    )
    safety_option_used = bool(
        arguments.attenuation_db is not None
        or arguments.tx_gain_db is not None
        or arguments.rx_gain_db is not None
        or arguments.i_understand_rf_output_is_enabled
        or arguments.i_confirm_txrx_to_rx2_loopback
        or arguments.i_confirm_atr_trm_pa_disabled
        or getattr(arguments, "i_confirm_trm_pa_disconnected", False)
        or getattr(arguments, "i_confirm_atr_cro_verified", False)
    )

    if not profile_selected:
        if safety_option_used:
            raise ValueError(
                "Loopback safety options require an explicit test profile"
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
        config["Stage3HLoopbackAtrActive"] = False
        config["EttusAtrAllowOverlapForSimulation"] = False
        config["EttusRfTargetEmulatorEnabled"] = False
        config["EttusRfTargetUseScenario"] = False
        return "RECEIVE_ONLY"

    _ApplyProfileGainDefaults(arguments)

    atr_enabled = bool(
        getattr(arguments, "stage3h_atr_loopback", False)
        or getattr(
            arguments,
            "stage3i_atr_rf_target_overlap",
            False,
        )
    )
    _ValidateCommonLoopbackArguments(
        arguments,
        atr_enabled=atr_enabled,
    )
    _ApplyCommonTimedLoopback(config, arguments)

    if atr_enabled:
        config.update({
            "EttusAtrGpioEnabled": True,
            "EttusAtrCroVerifiedAcknowledged": True,
            "EttusTrmPaDisconnectedConfirmed": True,
            "EttusGPIOBank": "FP0",
            "EttusTxAtrGPIO": 1,
            "EttusRxAtrGPIO": 2,
            "EttusAtrOverlapGPIO": 3,
            "EttusTxLeadingZeroSamples": 8,
            "Stage3HLoopbackAtrActive": True,
        })
        if getattr(
            arguments,
            "stage3i_atr_rf_target_overlap",
            False,
        ):
            if float(arguments.target_range_km) <= 0.0:
                raise ValueError("Target range must be positive")
            if int(arguments.loopback_hardware_delay_samples) < 0:
                raise ValueError(
                    "Loopback hardware delay must not be negative"
                )
            config.update({
                "EttusAtrAllowOverlapForSimulation": True,
                "EttusRfTargetEmulatorEnabled": True,
                "EttusRfTargetUseScenario": True,
                "EttusRfTargetRangeM": (
                    float(arguments.target_range_km) * 1000.0
                ),
                "EttusRfTargetBearingDeg": float(
                    arguments.target_bearing_deg
                ),
                "EttusRfTargetAngleHalfWidthDeg": 2.0,
                "EttusRfTargetRadialVelocityMps": float(
                    arguments.target_radial_velocity_mps
                ),
                "EttusLoopbackHardwareDelaySamples": int(
                    arguments.loopback_hardware_delay_samples
                ),
            })
            return "STAGE3I_ATR_RF_TARGET_OVERLAP"

        config["EttusAtrAllowOverlapForSimulation"] = False
        config["EttusRfTargetEmulatorEnabled"] = False
        config["EttusRfTargetUseScenario"] = False
        return "STAGE3H_ATR_LOOPBACK"

    if arguments.stage3f_rf_target:
        if float(arguments.target_range_km) <= 0.0:
            raise ValueError("Target range must be positive")
        if int(arguments.loopback_hardware_delay_samples) < 0:
            raise ValueError("Loopback hardware delay must not be negative")
        config.update({
            "EttusRfTargetEmulatorEnabled": True,
            "EttusRfTargetUseScenario": True,
            "EttusAtrAllowOverlapForSimulation": False,
            "EttusRfTargetRangeM": (
                float(arguments.target_range_km) * 1000.0
            ),
            "EttusRfTargetBearingDeg": float(
                arguments.target_bearing_deg
            ),
            "EttusRfTargetAngleHalfWidthDeg": 2.0,
            "EttusRfTargetRadialVelocityMps": float(
                arguments.target_radial_velocity_mps
            ),
            "EttusLoopbackHardwareDelaySamples": int(
                arguments.loopback_hardware_delay_samples
            ),
        })
        return "STAGE3F_RF_TARGET"

    config["EttusRfTargetEmulatorEnabled"] = False
    config["EttusRfTargetUseScenario"] = False
    config["Stage3HLoopbackAtrActive"] = False
    return "STAGE3E1_LOOPBACK"
