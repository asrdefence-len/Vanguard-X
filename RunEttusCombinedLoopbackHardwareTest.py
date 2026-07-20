"""Stage 3D guarded combined timed-TX/RX cable-loopback test.

RF connection: B200mini TX/RX -> attenuator -> B200mini RX2.
ATR, TRM and PA are deliberately not controlled by this program.
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


TX_EVENTS = {
    0x01: "burst_ack",
    0x02: "underflow",
    0x04: "sequence_error",
    0x08: "time_error",
    0x10: "underflow_in_packet",
    0x20: "sequence_error_in_burst",
}
RX_ERRORS = {
    0: "none", 1: "timeout", 2: "late_command", 4: "broken_chain",
    8: "overflow", 12: "alignment", 15: "bad_packet",
}


def numeric_value(value):
    raw = getattr(value, "value", value)
    try:
        return int(raw)
    except (TypeError, ValueError):
        return None


def parse_arguments(argv=None):
    parser = argparse.ArgumentParser(description="Guarded combined TX/RX loopback test")
    parser.add_argument("--serial", default="34A0320")
    parser.add_argument("--device-args", default="")
    parser.add_argument("--frequency-mhz", type=float, default=1000.0)
    parser.add_argument("--tx-gain-db", type=float, default=0.0)
    parser.add_argument("--rx-gain-db", type=float, default=10.0)
    parser.add_argument("--waveform", default="Frank10_20MHz")
    parser.add_argument("--prf-hz", type=float, default=4000.0)
    parser.add_argument("--pulses", type=int, default=128)
    parser.add_argument("--queue-depth", type=int, default=20)
    parser.add_argument("--lead-ms", type=float, default=50.0)
    parser.add_argument("--rx-start-us", type=float, default=6.0)
    parser.add_argument("--tx-test-offset-us", type=float, default=10.0)
    parser.add_argument("--tx-repetitions", type=int, default=1)
    parser.add_argument("--expected-hardware-delay-samples", type=int, default=0)
    parser.add_argument(
        "--pair-order", choices=("rx-first", "tx-first"),
        default="rx-first",
    )
    parser.add_argument("--rx-samples", type=int, default=4043)
    parser.add_argument("--timeout-sec", type=float, default=1.0)
    parser.add_argument("--attenuation-db", type=float, required=True)
    parser.add_argument("--minimum-peak-db", type=float, default=8.0)
    parser.add_argument("--capture-file", default="stage3d_loopback_capture.npz")
    parser.add_argument("--i-understand-rf-output-is-enabled", action="store_true")
    parser.add_argument("--i-confirm-txrx-to-rx2-loopback", action="store_true")
    return parser.parse_args(argv)


def validate_arguments(args):
    if not args.i_understand_rf_output_is_enabled:
        raise ValueError("Missing --i-understand-rf-output-is-enabled")
    if not args.i_confirm_txrx_to_rx2_loopback:
        raise ValueError("Missing --i-confirm-txrx-to-rx2-loopback")
    if args.attenuation_db < 30.0:
        raise ValueError("Stage 3D requires at least 30 dB attenuation")
    if not 0.0 <= args.tx_gain_db <= 70.0:
        raise ValueError("Stage 3D TX gain must be between 0 and 70 dB")
    if not 1000.0 <= args.prf_hz <= 4000.0:
        raise ValueError("PRF must be between 1000 and 4000 Hz")
    if args.pulses <= 0 or args.rx_samples <= 0:
        raise ValueError("Pulse and RX sample counts must be positive")
    if args.tx_repetitions <= 0:
        raise ValueError("TX repetitions must be positive")
    if not -1000 <= args.expected_hardware_delay_samples <= 1000:
        raise ValueError("Expected hardware delay must be within +/-1000 samples")
    if not 1 <= args.queue_depth <= 20:
        raise ValueError("Queue depth must be between 1 and 20")
    if args.lead_ms < 20.0:
        raise ValueError("Command lead must be at least 20 ms")
    if args.tx_test_offset_us <= args.rx_start_us:
        raise ValueError("Diagnostic TX must occur after RX starts")
    rx_duration_us = args.rx_samples / 40.0
    tx_duration_us = 5.0 * args.tx_repetitions
    if args.tx_test_offset_us + tx_duration_us >= args.rx_start_us + rx_duration_us:
        raise ValueError("Diagnostic TX waveform does not fit in RX window")
    if args.rx_start_us + rx_duration_us >= 1.0e6 / args.prf_hz:
        raise ValueError("RX window does not fit within PRI")


def device_args(args):
    return str(args.device_args).strip() or f"serial={args.serial}"


def tx_metadata(time_sec):
    metadata = uhd.types.TXMetadata()
    metadata.has_time_spec = True
    metadata.time_spec = uhd.types.TimeSpec(float(time_sec))
    metadata.start_of_burst = True
    metadata.end_of_burst = True
    return metadata


def issue_rx(streamer, samples, time_sec):
    command = uhd.types.StreamCMD(uhd.types.StreamMode.num_done)
    command.num_samps = int(samples)
    command.stream_now = False
    command.time_spec = uhd.types.TimeSpec(float(time_sec))
    streamer.issue_stream_cmd(command)


def receive_window(streamer, samples, timeout):
    metadata = uhd.types.RXMetadata()
    maximum = int(streamer.get_max_num_samps())
    packet = np.zeros((1, maximum), dtype=np.complex64)
    output = np.zeros(samples, dtype=np.complex64)
    received = 0
    first_time = None
    errors = []
    while received < samples:
        request = min(maximum, samples - received)
        count = int(streamer.recv(packet[:, :request], metadata, timeout))
        code = numeric_value(metadata.error_code)
        if code == 0:
            if count:
                if first_time is None and getattr(metadata, "has_time_spec", False):
                    first_time = float(metadata.time_spec.get_real_secs())
                output[received:received + count] = packet[0, :count]
                received += count
            continue
        errors.append(RX_ERRORS.get(code, f"unknown_{code}"))
        if code == 8:
            continue
        break
    return output, received, first_time, errors


def receive_burst_ack(tx_streamer, timeout):
    metadata = uhd.types.TXAsyncMetadata()
    deadline = time.monotonic() + timeout
    errors = []
    while time.monotonic() < deadline:
        remaining = max(0.0, deadline - time.monotonic())
        if not tx_streamer.recv_async_msg(metadata, remaining):
            break
        code = numeric_value(metadata.event_code)
        if code == 0x01:
            return True, errors
        errors.append(TX_EVENTS.get(code, f"unknown_{code}"))
    return False, errors


def drain_tx_events_nonblocking(tx_streamer, metadata, acknowledgements, errors):
    """Drain currently available TX events without delaying replenishment."""
    while tx_streamer.recv_async_msg(metadata, 0.0):
        code = numeric_value(metadata.event_code)
        if code == 0x01:
            acknowledgements += 1
        else:
            errors.append(TX_EVENTS.get(code, f"unknown_{code}"))
    return acknowledgements


def matched_filter_metrics(iq, waveform, expected_lag):
    correlation = np.abs(np.correlate(iq, waveform, mode="valid"))
    peak_lag = int(np.argmax(correlation))
    peak = float(correlation[peak_lag])
    guard = max(16, len(waveform) // 4)
    mask = np.ones(len(correlation), dtype=bool)
    mask[max(0, peak_lag-guard):min(len(mask), peak_lag+guard+1)] = False
    floor = float(np.median(correlation[mask])) if np.any(mask) else 0.0
    peak_db = 20.0 * np.log10(max(peak, 1e-20) / max(floor, 1e-20))
    return peak_lag, peak_db


def run(args):
    if uhd is None:
        raise RuntimeError("UHD Python bindings are unavailable") from _UHD_IMPORT_ERROR
    library = WaveformLibrary({"EttusSampleRateHz": 40.0e6})
    library.LoadDefaultWaveforms()
    waveform = np.asarray(library.Get(args.waveform), dtype=np.complex64)
    transmit_waveform = np.tile(waveform, args.tx_repetitions).astype(np.complex64)
    rate = 40.0e6
    pri = 1.0 / args.prf_hz
    rx_delay = args.rx_start_us * 1e-6
    tx_delay = args.tx_test_offset_us * 1e-6
    scheduled_lag = int(round((tx_delay-rx_delay)*rate))
    expected_lag = scheduled_lag+args.expected_hardware_delay_samples

    print("COMBINED TIMED RF TRANSMIT/RECEIVE IS ENABLED")
    print(f"  RF path:        TX/RX -> {args.attenuation_db:.1f} dB -> RX2")
    print("  ATR / TRM / PA: DISABLED (this program does not control them)")
    print(
        f"  waveform:       {args.waveform}, {len(waveform)} samples x "
        f"{args.tx_repetitions} = {len(transmit_waveform)} TX samples"
    )
    print(f"  PRF / pulses:   {args.prf_hz:.0f} Hz / {args.pulses}")
    print(f"  test timing:    RX T+{args.rx_start_us:.3f} us; TX T+{args.tx_test_offset_us:.3f} us")
    print(f"  scheduled lag:  {scheduled_lag} samples")
    print(
        f"  hardware delay: {args.expected_hardware_delay_samples:+d} samples; "
        f"expected physical lag={expected_lag}"
    )
    print(f"  queue:          bounded matched pairs, depth={args.queue_depth}")
    print(f"  pair order:     {args.pair_order}")

    usrp = uhd.usrp.MultiUSRP(device_args(args))
    channel = 0
    frequency = args.frequency_mhz * 1e6
    usrp.set_rx_rate(rate, channel)
    usrp.set_tx_rate(rate, channel)
    usrp.set_rx_freq(uhd.types.TuneRequest(frequency), channel)
    usrp.set_tx_freq(uhd.types.TuneRequest(frequency), channel)
    usrp.set_rx_gain(args.rx_gain_db, channel)
    usrp.set_tx_gain(args.tx_gain_db, channel)
    usrp.set_rx_antenna("RX2", channel)
    usrp.set_tx_antenna("TX/RX", channel)
    actual_tx_gain = float(usrp.get_tx_gain(channel))
    actual_rx_gain = float(usrp.get_rx_gain(channel))
    if not np.isclose(actual_tx_gain, args.tx_gain_db, rtol=0, atol=0.26):
        raise RuntimeError(
            f"UHD coerced TX gain to {actual_tx_gain:.2f} dB; "
            f"requested {args.tx_gain_db:.2f} dB"
        )
    print(f"  RF selections:  TX={usrp.get_tx_antenna(channel)}, RX={usrp.get_rx_antenna(channel)}")
    print(f"  actual gains:   TX={actual_tx_gain:.2f} dB, RX={actual_rx_gain:.2f} dB")
    rx_args = uhd.usrp.StreamArgs("fc32", "sc16")
    tx_args = uhd.usrp.StreamArgs("fc32", "sc16")
    rx_args.channels = [channel]
    tx_args.channels = [channel]
    rx = usrp.get_rx_stream(rx_args)
    tx = usrp.get_tx_stream(tx_args)

    # Prime only the receive data path before establishing the timed dwell.
    warm = uhd.types.StreamCMD(uhd.types.StreamMode.num_done)
    warm.num_samps = args.rx_samples
    warm.stream_now = True
    rx.issue_stream_cmd(warm)
    _, warm_count, _, warm_errors = receive_window(rx, args.rx_samples, args.timeout_sec)
    if warm_count != args.rx_samples or warm_errors:
        raise RuntimeError(f"RX warm-up failed: {warm_count}/{args.rx_samples}, {warm_errors}")

    t0 = float(usrp.get_time_now().get_real_secs()) + args.lead_ms / 1000.0
    next_pair = 0
    collected = 0
    maximum_outstanding = 0
    tx_errors = []
    tx_acknowledgements = 0
    rx_errors = []
    timestamp_errors = []
    lag_errors = []
    peak_db_values = []
    captured_iq = []
    async_metadata = uhd.types.TXAsyncMetadata()

    def queue_pair(index):
        base = t0 + index * pri
        def send_tx():
            return int(tx.send(
                transmit_waveform,
                tx_metadata(base + tx_delay),
                args.timeout_sec,
            ))

        if args.pair_order == "tx-first":
            # Proven S-band diagnostic ordering: enqueue future TX, then arm
            # the earlier RX window. Both operations remain hardware-timed.
            sent = send_tx()
            issue_rx(rx, args.rx_samples, base + rx_delay)
        else:
            issue_rx(rx, args.rx_samples, base + rx_delay)
            sent = send_tx()
        if sent != len(transmit_waveform):
            raise RuntimeError(f"Pulse {index}: short TX send {sent}/{len(transmit_waveform)}")

    initial = min(args.queue_depth, args.pulses)
    for _ in range(initial):
        queue_pair(next_pair)
        next_pair += 1
    maximum_outstanding = initial

    for pulse in range(args.pulses):
        iq, count, first_time, errors = receive_window(rx, args.rx_samples, args.timeout_sec)
        rx_errors.extend(f"pulse {pulse}: {item}" for item in errors)
        if count != args.rx_samples:
            rx_errors.append(f"pulse {pulse}: samples {count}/{args.rx_samples}")
        collected += 1

        # Replenish first. No diagnostic formatting, TX-event wait, IQ copy or
        # matched filtering is allowed to consume the scheduling horizon.
        if next_pair < args.pulses:
            queue_pair(next_pair)
            next_pair += 1
            maximum_outstanding = max(maximum_outstanding, next_pair-collected)

        tx_acknowledgements = drain_tx_events_nonblocking(
            tx, async_metadata, tx_acknowledgements, tx_errors
        )

        if first_time is not None:
            scheduled_rx = t0 + pulse * pri + rx_delay
            timestamp_errors.append(first_time - scheduled_rx)
        if count == args.rx_samples:
            captured_iq.append(iq.copy())

    # All timing-critical scheduling is complete. Wait for the remaining TX
    # events once, using one overall deadline rather than one timeout per pulse.
    event_deadline = time.monotonic() + args.timeout_sec
    while tx_acknowledgements < args.pulses and time.monotonic() < event_deadline:
        remaining = max(0.0, event_deadline-time.monotonic())
        if not tx.recv_async_msg(async_metadata, remaining):
            break
        code = numeric_value(async_metadata.event_code)
        if code == 0x01:
            tx_acknowledgements += 1
        else:
            tx_errors.append(TX_EVENTS.get(code, f"unknown_{code}"))

    if tx_acknowledgements != args.pulses:
        tx_errors.append(
            f"burst acknowledgements {tx_acknowledgements}/{args.pulses}"
        )

    # Processing is deliberately deferred until after the complete dwell.
    capture_array = np.asarray(captured_iq, dtype=np.complex64)
    np.savez_compressed(
        args.capture_file,
        iq=capture_array,
        waveform=waveform,
        transmit_waveform=transmit_waveform,
        tx_repetitions=np.int64(args.tx_repetitions),
        sample_rate_hz=np.float64(rate),
        prf_hz=np.float64(args.prf_hz),
        rx_start_us=np.float64(args.rx_start_us),
        tx_test_offset_us=np.float64(args.tx_test_offset_us),
        expected_lag=np.int64(expected_lag),
        scheduled_lag=np.int64(scheduled_lag),
        expected_hardware_delay_samples=np.int64(
            args.expected_hardware_delay_samples
        ),
        attenuation_db=np.float64(args.attenuation_db),
        rx_gain_db=np.float64(args.rx_gain_db),
        tx_gain_db=np.float64(args.tx_gain_db),
        pair_order=np.asarray(args.pair_order),
    )
    print(f"  capture file:   {args.capture_file}")

    for iq in captured_iq:
        lag, peak_db = matched_filter_metrics(iq, waveform, expected_lag)
        lag_errors.append(lag - expected_lag)
        peak_db_values.append(peak_db)

    transport_failures = tx_errors + rx_errors
    signal_failures = []
    if peak_db_values:
        minimum_peak = min(peak_db_values)
        maximum_lag_error = max(abs(value) for value in lag_errors)
        if minimum_peak < args.minimum_peak_db:
            signal_failures.append(f"minimum matched-filter peak {minimum_peak:.2f} dB is below {args.minimum_peak_db:.2f} dB")
        if args.tx_repetitions == 1 and maximum_lag_error > 16:
            signal_failures.append(f"maximum matched-filter lag error is {maximum_lag_error} samples")
        elif args.tx_repetitions > 1:
            valid_lag_end = expected_lag + (args.tx_repetitions-1)*len(waveform)
            peak_lags = np.asarray(lag_errors) + expected_lag
            inside = np.logical_and(peak_lags >= expected_lag-16, peak_lags <= valid_lag_end+16)
            if not np.all(inside):
                signal_failures.append(
                    f"{int(np.count_nonzero(~inside))} pulse peaks lie outside "
                    "the repeated diagnostic burst"
                )
    else:
        signal_failures.append("no complete RX windows available for matched filtering")

    print(f"  TX sends/ACKs:  {next_pair}/{args.pulses} / {tx_acknowledgements}/{args.pulses}")
    print(f"  RX windows:     {args.pulses-len([e for e in rx_errors if 'samples' in e])}/{args.pulses}")
    print(f"  max outstanding:{maximum_outstanding:4d}")
    if timestamp_errors:
        values = np.asarray(timestamp_errors)*1e9
        print(f"  RX time error:  {values.min():+.1f} / {values.mean():+.1f} / {values.max():+.1f} ns")
    if peak_db_values:
        print(f"  MF peak:        {min(peak_db_values):.2f} / {np.mean(peak_db_values):.2f} / {max(peak_db_values):.2f} dB min/mean/max")
        print(f"  lag error:      {min(lag_errors):+d} / {max(lag_errors):+d} samples min/max")
    for failure in transport_failures:
        print(f"  FAILURE: {failure}")
    for failure in signal_failures:
        print(f"  SIGNAL FAILURE: {failure}")
    print(
        "  transport:      "
        f"{'PASS' if not transport_failures else 'FAIL'}"
    )
    print(
        "  signal lock:    "
        f"{'PASS' if not signal_failures else 'FAIL'}"
    )
    passed = not transport_failures and not signal_failures
    print(f"Combined loopback hardware test {'PASSED' if passed else 'FAILED'}")
    return 0 if passed else 1


def main(argv=None):
    args = parse_arguments(argv)
    try:
        validate_arguments(args)
        return run(args)
    except Exception as exc:
        print(f"Combined loopback hardware test FAILED: {exc}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
