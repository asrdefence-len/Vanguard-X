"""Stage 3C: guarded, low-power, timed-burst TX test for an Ettus B200mini.

This test deliberately does not enable ATR GPIO, a TRM, or a power amplifier.
Connect the RF port through a suitably rated attenuator to a 50-ohm load.
"""

from __future__ import annotations

import argparse
import sys
import time

import numpy as np

from WaveformLibrary import WaveformLibrary

try:
    import uhd
except ImportError as exc:
    uhd = None
    _UHD_IMPORT_ERROR = exc
else:
    _UHD_IMPORT_ERROR = None


EVENT_NAMES = {
    0x01: "burst_ack",
    0x02: "underflow",
    0x04: "sequence_error",
    0x08: "time_error",
    0x10: "underflow_in_packet",
    0x20: "sequence_error_in_burst",
}


def event_value(event_code):
    raw = getattr(event_code, "value", event_code)
    try:
        return int(raw)
    except (TypeError, ValueError):
        return None


def parse_arguments(argv=None):
    parser = argparse.ArgumentParser(
        description="Guarded low-power timed-TX queue test",
    )
    parser.add_argument("--serial", default="34A0320")
    parser.add_argument("--device-args", default="")
    parser.add_argument("--frequency-mhz", type=float, default=1000.0)
    parser.add_argument("--gain-db", type=float, default=0.0)
    parser.add_argument("--waveform", default="Frank10_20MHz")
    parser.add_argument("--prf-hz", type=float, default=4000.0)
    parser.add_argument("--pulses", type=int, default=128)
    parser.add_argument("--queue-depth", type=int, default=20)
    parser.add_argument("--lead-ms", type=float, default=50.0)
    parser.add_argument("--timeout-sec", type=float, default=1.0)
    parser.add_argument("--attenuation-db", type=float, required=True)
    parser.add_argument(
        "--i-understand-rf-output-is-enabled",
        action="store_true",
        help="required safety acknowledgement",
    )
    return parser.parse_args(argv)


def validate_arguments(args):
    if not args.i_understand_rf_output_is_enabled:
        raise ValueError(
            "Refusing to transmit without --i-understand-rf-output-is-enabled"
        )
    if args.attenuation_db < 30.0:
        raise ValueError("Initial Stage 3C testing requires at least 30 dB attenuation")
    if not 1000.0 <= args.prf_hz <= 4000.0:
        raise ValueError("PRF must be between 1000 and 4000 Hz")
    if args.pulses <= 0:
        raise ValueError("Pulse count must be positive")
    if not 1 <= args.queue_depth <= 20:
        raise ValueError("Queue depth must be between 1 and 20")
    if args.lead_ms < 20.0:
        raise ValueError("Command lead must be at least 20 ms")
    if args.gain_db != 0.0:
        raise ValueError("Initial Stage 3C testing fixes TX gain at 0 dB")


def build_device_args(args):
    explicit = str(args.device_args).strip()
    return explicit if explicit else f"serial={args.serial}"


def make_tx_metadata(time_sec):
    metadata = uhd.types.TXMetadata()
    metadata.has_time_spec = True
    metadata.time_spec = uhd.types.TimeSpec(float(time_sec))
    metadata.start_of_burst = True
    metadata.end_of_burst = True
    return metadata


def run_test(args):
    if uhd is None:
        raise RuntimeError("UHD Python bindings are unavailable") from _UHD_IMPORT_ERROR

    config = {"EttusSampleRateHz": 40.0e6}
    library = WaveformLibrary(config)
    library.LoadDefaultWaveforms()
    waveform = np.asarray(library.Get(args.waveform), dtype=np.complex64)
    sample_rate = 40.0e6
    pri_sec = 1.0 / float(args.prf_hz)
    pulse_duration = len(waveform) / sample_rate
    if pulse_duration >= pri_sec:
        raise ValueError("Waveform does not fit within the selected PRI")

    print("TIMED RF TRANSMIT IS ENABLED FOR THIS TEST")
    print(f"  RF path:        attenuator={args.attenuation_db:.1f} dB into 50 ohm")
    print("  ATR / TRM / PA: DISABLED (this program does not control them)")
    print(f"  waveform:       {args.waveform}, {len(waveform)} samples, {pulse_duration*1e6:.3f} us")
    print(f"  PRF / pulses:   {args.prf_hz:.0f} Hz / {args.pulses}")
    print(f"  queue:          bounded timed bursts, depth={args.queue_depth}")
    print(f"  TX gain:        {args.gain_db:.1f} dB")

    usrp = uhd.usrp.MultiUSRP(build_device_args(args))
    channel = 0
    usrp.set_tx_rate(sample_rate, channel)
    usrp.set_tx_freq(uhd.types.TuneRequest(args.frequency_mhz * 1.0e6), channel)
    usrp.set_tx_gain(args.gain_db, channel)
    stream_args = uhd.usrp.StreamArgs("fc32", "sc16")
    stream_args.channels = [channel]
    tx = usrp.get_tx_stream(stream_args)

    actual_rate = float(usrp.get_tx_rate(channel))
    actual_gain = float(usrp.get_tx_gain(channel))
    if not np.isclose(actual_rate, sample_rate, rtol=0.0, atol=1.0):
        raise RuntimeError(f"TX sample rate is {actual_rate:g}, expected {sample_rate:g}")
    if actual_gain > 0.01:
        raise RuntimeError(f"TX gain is {actual_gain:g} dB, expected minimum 0 dB")

    t0 = float(usrp.get_time_now().get_real_secs()) + args.lead_ms / 1000.0
    next_pulse = 0
    acknowledged = 0
    maximum_outstanding = 0
    errors = []

    def queue_one(index):
        scheduled = t0 + index * pri_sec
        sent = int(tx.send(waveform, make_tx_metadata(scheduled), args.timeout_sec))
        if sent != len(waveform):
            raise RuntimeError(
                f"Pulse {index}: short TX send {sent}/{len(waveform)} samples"
            )

    initial = min(args.queue_depth, args.pulses)
    for _ in range(initial):
        queue_one(next_pulse)
        next_pulse += 1
    maximum_outstanding = initial

    async_metadata = uhd.types.TXAsyncMetadata()
    deadline = time.monotonic() + args.lead_ms / 1000.0 + args.pulses * pri_sec + 5.0
    while acknowledged < args.pulses:
        if time.monotonic() > deadline:
            errors.append(f"timeout waiting for burst acknowledgements ({acknowledged}/{args.pulses})")
            break
        if not tx.recv_async_msg(async_metadata, args.timeout_sec):
            continue
        code = event_value(async_metadata.event_code)
        name = EVENT_NAMES.get(code, f"unknown_{code}")
        if code != 0x01:
            errors.append(f"TX async event: {name} ({code})")
            continue
        acknowledged += 1
        if next_pulse < args.pulses:
            queue_one(next_pulse)
            next_pulse += 1
            maximum_outstanding = max(
                maximum_outstanding,
                next_pulse - acknowledged,
            )

    print(f"  sends:          {next_pulse}/{args.pulses} complete")
    print(f"  burst ACKs:     {acknowledged}/{args.pulses}")
    print(f"  max outstanding:{maximum_outstanding:4d}")
    if errors:
        for error in errors:
            print(f"  FAILURE: {error}")
        print("Timed transmit hardware test FAILED")
        return 1
    print("Timed transmit hardware test PASSED")
    return 0


def main(argv=None):
    args = parse_arguments(argv)
    try:
        validate_arguments(args)
        return run_test(args)
    except Exception as exc:
        print(f"Timed transmit hardware test FAILED: {exc}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
