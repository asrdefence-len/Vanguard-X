"""Guarded TX-only B200mini burst generator for oscilloscope inspection."""

import argparse
import sys
import time

import numpy as np

try:
    import uhd
except ImportError as exc:
    uhd = None
    IMPORT_ERROR = exc
else:
    IMPORT_ERROR = None


EVENTS = {
    0x01: "burst_ack", 0x02: "underflow", 0x04: "sequence_error",
    0x08: "time_error", 0x10: "underflow_in_packet",
    0x20: "sequence_error_in_burst",
}


def value(code):
    try: return int(getattr(code, "value", code))
    except (TypeError, ValueError): return None


def arguments(argv=None):
    p = argparse.ArgumentParser(description="TX-only CRO burst generator")
    p.add_argument("--serial", default="34A0320")
    p.add_argument("--centre-frequency-mhz", type=float, default=99.0)
    p.add_argument("--baseband-tone-mhz", type=float, default=1.0)
    p.add_argument("--sample-rate-msps", type=float, default=5.0)
    p.add_argument("--burst-us", type=float, default=50.0)
    p.add_argument("--prf-hz", type=float, default=100.0)
    p.add_argument("--bursts", type=int, default=100)
    p.add_argument("--tx-gain-db", type=float, default=0.0)
    p.add_argument("--attenuation-db", type=float, required=True)
    p.add_argument("--queue-depth", type=int, default=20)
    p.add_argument("--lead-ms", type=float, default=100.0)
    p.add_argument("--i-understand-rf-output-is-enabled", action="store_true")
    p.add_argument("--i-confirm-scope-is-50-ohm-terminated", action="store_true")
    return p.parse_args(argv)


def validate(a):
    if not a.i_understand_rf_output_is_enabled:
        raise ValueError("Missing RF-output acknowledgement")
    if not a.i_confirm_scope_is_50_ohm_terminated:
        raise ValueError("Missing 50-ohm scope-termination acknowledgement")
    if a.attenuation_db < 30:
        raise ValueError("Initial CRO test requires at least 30 dB attenuation")
    if a.tx_gain_db != 0:
        raise ValueError("Initial CRO test fixes TX gain at 0 dB")
    rate = a.sample_rate_msps*1e6
    tone = a.baseband_tone_mhz*1e6
    if rate <= 0 or abs(tone) >= rate/2:
        raise ValueError("Baseband tone must be inside the Nyquist bandwidth")
    if not 1 <= a.queue_depth <= 20:
        raise ValueError("Queue depth must be between 1 and 20")
    if a.bursts <= 0 or a.prf_hz <= 0 or a.burst_us <= 0:
        raise ValueError("Burst count, PRF and duration must be positive")
    if a.burst_us*1e-6 >= 1/a.prf_hz:
        raise ValueError("Burst duration must be shorter than PRI")
    if a.lead_ms < 50:
        raise ValueError("Command lead must be at least 50 ms")


def run(a):
    if uhd is None:
        raise RuntimeError("UHD Python bindings unavailable") from IMPORT_ERROR
    rate = a.sample_rate_msps*1e6
    tone = a.baseband_tone_mhz*1e6
    samples = int(round(a.burst_us*1e-6*rate))
    n = np.arange(samples, dtype=np.float64)
    waveform = (0.5*np.exp(2j*np.pi*tone*n/rate)).astype(np.complex64)

    usrp = uhd.usrp.MultiUSRP(f"serial={a.serial}")
    channel = 0
    usrp.set_tx_rate(rate, channel)
    usrp.set_tx_freq(
        uhd.types.TuneRequest(a.centre_frequency_mhz*1e6), channel
    )
    usrp.set_tx_gain(a.tx_gain_db, channel)
    usrp.set_tx_antenna("TX/RX", channel)
    stream_args = uhd.usrp.StreamArgs("fc32", "sc16")
    stream_args.channels = [channel]
    tx = usrp.get_tx_stream(stream_args)

    t0 = float(usrp.get_time_now().get_real_secs()) + a.lead_ms/1000
    pri = 1/a.prf_hz
    queued = 0
    acknowledged = 0
    errors = []
    depth = min(a.queue_depth, a.bursts)

    def send(index):
        md = uhd.types.TXMetadata()
        md.has_time_spec = True
        md.time_spec = uhd.types.TimeSpec(t0+index*pri)
        md.start_of_burst = True
        md.end_of_burst = True
        count = int(tx.send(waveform, md, 1.0))
        if count != samples:
            raise RuntimeError(f"Burst {index}: short send {count}/{samples}")

    for _ in range(depth):
        send(queued); queued += 1

    md = uhd.types.TXAsyncMetadata()
    deadline = time.monotonic()+a.lead_ms/1000+a.bursts*pri+2
    while acknowledged < a.bursts and time.monotonic() < deadline:
        if not tx.recv_async_msg(md, 0.25):
            continue
        code = value(md.event_code)
        if code == 0x01:
            acknowledged += 1
            if queued < a.bursts:
                send(queued); queued += 1
        else:
            errors.append(EVENTS.get(code, f"unknown_{code}"))

    carrier = a.centre_frequency_mhz+a.baseband_tone_mhz
    print("TX-ONLY CRO TEST")
    print(f"  connection:     TX/RX -> {a.attenuation_db:.1f} dB -> 50-ohm CRO input")
    print(f"  RF centre:      {a.centre_frequency_mhz:.3f} MHz")
    print(f"  baseband tone:  {a.baseband_tone_mhz:+.3f} MHz")
    print(f"  CRO carrier:    approximately {carrier:.3f} MHz")
    print(f"  burst:          {a.burst_us:.3f} us, {samples} samples")
    print(f"  PRF / count:    {a.prf_hz:.3f} Hz / {a.bursts}")
    print(f"  sends / ACKs:   {queued}/{a.bursts} / {acknowledged}/{a.bursts}")
    print(f"  TX events:      {errors}")
    passed = queued == a.bursts and acknowledged == a.bursts and not errors
    print(f"CRO transmit test {'PASSED' if passed else 'FAILED'}")
    return 0 if passed else 1


def main(argv=None):
    a = arguments(argv)
    try:
        validate(a)
        return run(a)
    except Exception as exc:
        print(f"CRO transmit test FAILED: {exc}")
        return 1


if __name__ == "__main__": sys.exit(main())
