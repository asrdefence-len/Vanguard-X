"""Guarded Stage 3E1 operational-source timed-pair transport test.

RF connection: B200mini TX/RX -> at least 30 dB attenuator -> B200mini RX2.
ATR, TRM and PA are deliberately disabled. Signal detection is not an
acceptance criterion because operational RX starts after the transmitted pulse;
the test verifies finite TX/RX command pairing and transport completion.
"""

from __future__ import annotations

import argparse
import sys
import traceback

import numpy as np

from EttusRadarSource import EttusRadarSource
from RadarPlans import make_uniform_dwell_plan
from RadarTiming import CalculateRadarTiming
from WaveformLibrary import WaveformLibrary


def parse_arguments(argv=None):
    parser = argparse.ArgumentParser(
        description="Run guarded Stage 3E1 operational timed TX/RX pairs",
    )
    parser.add_argument("--serial", default="34A0320")
    parser.add_argument("--device-args", default="")
    parser.add_argument("--frequency-mhz", type=float, default=1000.0)
    parser.add_argument("--tx-gain-db", type=float, default=0.0)
    parser.add_argument("--rx-gain-db", type=float, default=10.0)
    parser.add_argument("--waveform", default="Frank10_20MHz")
    parser.add_argument("--prf-hz", type=float, default=1000.0)
    parser.add_argument("--pulses", type=int, default=32)
    parser.add_argument("--maximum-range-km", type=float, default=15.0)
    parser.add_argument("--queue-depth", type=int, default=1)
    parser.add_argument("--lead-ms", type=float, default=50.0)
    parser.add_argument("--timeout-sec", type=float, default=1.0)
    parser.add_argument("--dwells", type=int, default=1)
    parser.add_argument("--attenuation-db", type=float, required=True)
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


def validate_arguments(args):
    if not args.i_understand_rf_output_is_enabled:
        raise ValueError(
            "Missing --i-understand-rf-output-is-enabled"
        )
    if not args.i_confirm_txrx_to_rx2_loopback:
        raise ValueError(
            "Missing --i-confirm-txrx-to-rx2-loopback"
        )
    if not args.i_confirm_atr_trm_pa_disabled:
        raise ValueError("Missing --i-confirm-atr-trm-pa-disabled")
    if args.attenuation_db < 30.0:
        raise ValueError("Stage 3E1 requires at least 30 dB attenuation")
    if not 0.0 <= args.tx_gain_db <= 50.0:
        raise ValueError("Stage 3E1 TX gain must be between 0 and 50 dB")
    if not 1000.0 <= args.prf_hz <= 4000.0:
        raise ValueError("PRF must be between 1000 and 4000 Hz")
    if args.pulses <= 0 or args.dwells <= 0:
        raise ValueError("Pulse and dwell counts must be positive")
    if not 1 <= args.queue_depth <= 20:
        raise ValueError("Queue depth must be between 1 and 20")
    if args.lead_ms < 20.0:
        raise ValueError("Command lead must be at least 20 ms")


def build_config(args):
    config = {
        "EttusOperatingMode": "TIMED_TX_RX",
        "EttusTimedTransmitEnabled": True,
        "EttusRfOutputAcknowledged": True,
        "EttusLoopbackConfirmed": True,
        "EttusExternalAttenuationDb": float(args.attenuation_db),
        "EttusAtrGpioEnabled": False,
        "EttusSerial": str(args.serial),
        "EttusRxFrequencyHz": float(args.frequency_mhz) * 1.0e6,
        "EttusTxFrequencyHz": float(args.frequency_mhz) * 1.0e6,
        "EttusRxGainDb": float(args.rx_gain_db),
        "EttusTxGainDb": float(args.tx_gain_db),
        "EttusRxAntenna": "RX2",
        "EttusTxAntenna": "TX/RX",
        "EttusRxChannel": 0,
        "EttusTxChannel": 0,
        "EttusSampleRateHz": 40.0e6,
        "EttusCpuFormat": "fc32",
        "EttusWireFormat": "sc16",
        "EttusReceiveTimeoutSec": float(args.timeout_sec),
        "EttusTxSendTimeoutSec": float(args.timeout_sec),
        "EttusTxAsyncTimeoutSec": float(args.timeout_sec),
        "EttusCommandLeadTimeSec": float(args.lead_ms) / 1000.0,
        "EttusCommandQueueDepth": int(args.queue_depth),
        "EttusRxWarmupEnabled": True,
        "RfFrequency": 9.4e9,
    }
    if str(args.device_args).strip():
        config["EttusDeviceArgs"] = str(args.device_args).strip()
    return config


def build_timing(library, args):
    return CalculateRadarTiming(
        library.GetMetadata(args.waveform),
        SelectedPrfHz=float(args.prf_hz),
        PulsesPerCpi=int(args.pulses),
        MaximumRangeM=float(args.maximum_range_km) * 1000.0,
        ReceiverRecoveryTimeSec=1.0e-6,
        RxEndMarginSec=2.0e-6,
        NextTxGuardTimeSec=2.0e-6,
    )


def build_plan(timing, dwell_id):
    return make_uniform_dwell_plan(
        dwell_id=int(dwell_id),
        task_id=0,
        task_type="STAGE3E1_TIMED_PAIR_TEST",
        waveform_id=timing.WaveformId,
        sample_rate=timing.SampleRateHz,
        num_samples=timing.NumRxSamples,
        num_pulses=timing.PulsesPerCpi,
        pri_sec=timing.PriSec,
        rx_start_delay_sec=timing.RxStartDelaySec,
        metadata={
            "Stage3E1HardwareTest": True,
            "RadarTiming": timing.ToMetadata(),
        },
    )


def print_plan(timing, args):
    print("STAGE 3E1 OPERATIONAL TIMED TX/RX IS ENABLED")
    print(
        f"  RF path:        TX/RX -> {args.attenuation_db:.1f} dB -> RX2"
    )
    print("  ATR / TRM / PA: DISABLED")
    print(f"  waveform:       {timing.WaveformId}")
    print(f"  sample rate:    {timing.SampleRateHz / 1e6:.3f} MS/s")
    print(
        f"  PRF / PRI:      {timing.SelectedPrfHz:.0f} Hz / "
        f"{timing.PriSec * 1e6:.3f} us"
    )
    print(
        f"  pulses / CPI:   {timing.PulsesPerCpi} / "
        f"{timing.CpiDurationSec * 1e3:.3f} ms"
    )
    print(
        f"  TX / RX start:  0.000 / "
        f"{timing.RxStartDelaySec * 1e6:.3f} us"
    )
    print(f"  RX samples:     {timing.NumRxSamples} per pulse")
    print(
        f"  bounded pairs:  depth={args.queue_depth}, "
        f"horizon={args.queue_depth * timing.PriSec * 1e3:.3f} ms"
    )
    print(f"  command lead:   {args.lead_ms:.3f} ms")
    print(f"  TX/RX gains:    {args.tx_gain_db:.1f} / {args.rx_gain_db:.1f} dB")
    print("  signal lock:    not required; transport test only")


def evaluate(raw, timing, queue_depth):
    diagnostics = dict(raw.Diagnostics or {})
    pulse_diagnostics = list(diagnostics.get("PulseDiagnostics", []))
    failures = []

    expected_shape = (timing.PulsesPerCpi, timing.NumRxSamples)
    if tuple(raw.IQ.shape) != expected_shape:
        failures.append(f"IQ shape {raw.IQ.shape}; expected {expected_shape}")
    if not np.all(np.asarray(raw.PulseValid, dtype=bool)):
        failures.append("One or more RX pulses are invalid")
    if diagnostics.get("OperatingMode") != "TIMED_TX_RX":
        failures.append("Source did not report TIMED_TX_RX")
    if diagnostics.get("TimedTransmitEnabled") is not True:
        failures.append("Source did not report timed transmit enabled")
    if diagnostics.get("ReceiveOnly") is not False:
        failures.append("Source incorrectly reported receive-only")

    tx_commands = int(diagnostics.get("TransmitCommandCount", 0))
    tx_acks = int(
        diagnostics.get("TransmitBurstAcknowledgementCount", 0)
    )
    if tx_commands != timing.PulsesPerCpi:
        failures.append(
            f"TX commands {tx_commands}/{timing.PulsesPerCpi}"
        )
    if tx_acks != tx_commands:
        failures.append(f"TX burst ACKs {tx_acks}/{tx_commands}")
    if diagnostics.get("TransmitEventErrors"):
        failures.append(
            f"TX events: {diagnostics.get('TransmitEventErrors')}"
        )

    maximum_pairs = int(
        diagnostics.get("MaximumOutstandingTimedPairs", 0)
    )
    expected_depth = min(int(queue_depth), timing.PulsesPerCpi)
    if maximum_pairs != expected_depth:
        failures.append(
            f"Maximum outstanding pairs {maximum_pairs}; "
            f"expected {expected_depth}"
        )

    rx_error_count = sum(
        int(item.get(name, 0))
        for item in pulse_diagnostics
        for name in (
            "TimeoutCount",
            "OverflowCount",
            "LateCommandCount",
            "BrokenChainCount",
            "AlignmentErrorCount",
            "BadPacketCount",
            "OtherErrorCount",
        )
    )
    if rx_error_count:
        failures.append(f"UHD reported {rx_error_count} RX errors")

    received_samples = sum(
        int(item.get("ReceivedSamples", 0))
        for item in pulse_diagnostics
    )
    expected_samples = timing.PulsesPerCpi * timing.NumRxSamples
    if received_samples != expected_samples:
        failures.append(
            f"RX samples {received_samples}/{expected_samples}"
        )

    return failures, tx_commands, tx_acks, received_samples, maximum_pairs


def run(args):
    config = build_config(args)
    library = WaveformLibrary(config)
    library.LoadDefaultWaveforms()
    timing = build_timing(library, args)
    print_plan(timing, args)

    source = EttusRadarSource(config, library)
    try:
        source.Initialise()
        for dwell_index in range(1, args.dwells + 1):
            raw = source.ExecuteDwell(build_plan(timing, dwell_index))
            (
                failures,
                tx_commands,
                tx_acks,
                received_samples,
                maximum_pairs,
            ) = evaluate(raw, timing, args.queue_depth)
            status = "PASS" if not failures else "FAIL"
            print(f"Dwell {dwell_index}: {status}")
            print(f"  IQ shape:       {raw.IQ.shape}")
            print(
                f"  TX sends/ACKs:  {tx_commands}/{timing.PulsesPerCpi} / "
                f"{tx_acks}/{timing.PulsesPerCpi}"
            )
            print(
                f"  RX samples:     {received_samples}/"
                f"{timing.PulsesPerCpi * timing.NumRxSamples}"
            )
            print(f"  max pairs:      {maximum_pairs}")
            print(
                f"  capture wall:   "
                f"{raw.Diagnostics.get('CaptureElapsedSec', 0.0) * 1e3:.3f} ms"
            )
            for failure in failures:
                print(f"  FAILURE: {failure}")
            if failures:
                return 1
    finally:
        source.Shutdown()

    print("Stage 3E1 operational timed-pair hardware test PASSED")
    return 0


def main(argv=None):
    args = parse_arguments(argv)
    try:
        validate_arguments(args)
        return run(args)
    except Exception:
        print("Stage 3E1 operational timed-pair hardware test FAILED")
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())
