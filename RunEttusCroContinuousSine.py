"""Minimal TX-only continuous sine-wave test for a 50-ohm oscilloscope."""

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


def event_value(code):
    try: return int(getattr(code, "value", code))
    except (TypeError, ValueError): return None


def parse(argv=None):
    p = argparse.ArgumentParser(description="TX-only continuous CRO sine test")
    p.add_argument("--serial", default="34A0320")
    p.add_argument("--centre-frequency-mhz", type=float, default=99.0)
    p.add_argument("--baseband-tone-mhz", type=float, default=1.0)
    p.add_argument("--sample-rate-msps", type=float, default=5.0)
    p.add_argument("--duration-sec", type=float, default=10.0)
    p.add_argument("--tx-gain-db", type=float, default=0.0)
    p.add_argument("--attenuation-db", type=float, required=True)
    p.add_argument("--i-understand-rf-output-is-enabled", action="store_true")
    p.add_argument("--i-confirm-scope-is-50-ohm-terminated", action="store_true")
    return p.parse_args(argv)


def validate(a):
    if not a.i_understand_rf_output_is_enabled:
        raise ValueError("Missing RF-output acknowledgement")
    if not a.i_confirm_scope_is_50_ohm_terminated:
        raise ValueError("Missing 50-ohm scope acknowledgement")
    if a.attenuation_db < 30:
        raise ValueError("This initial test requires at least 30 dB attenuation")
    if a.tx_gain_db != 0:
        raise ValueError("This initial test fixes TX gain at 0 dB")
    rate = a.sample_rate_msps*1e6
    tone = a.baseband_tone_mhz*1e6
    if rate <= 0 or not 0 < abs(tone) < rate/2:
        raise ValueError("Baseband tone must be nonzero and inside Nyquist")
    if not 0.1 <= a.duration_sec <= 120:
        raise ValueError("Duration must be between 0.1 and 120 seconds")


def run(a):
    if uhd is None:
        raise RuntimeError("UHD Python bindings unavailable") from IMPORT_ERROR
    requested_rate = a.sample_rate_msps*1e6
    tone = a.baseband_tone_mhz*1e6
    usrp = uhd.usrp.MultiUSRP(f"serial={a.serial}")
    usrp.set_tx_rate(requested_rate, 0)
    usrp.set_tx_freq(uhd.types.TuneRequest(a.centre_frequency_mhz*1e6), 0)
    usrp.set_tx_gain(a.tx_gain_db, 0)
    usrp.set_tx_antenna("TX/RX", 0)
    actual_rate = float(usrp.get_tx_rate(0))
    actual_frequency = float(usrp.get_tx_freq(0))
    actual_gain = float(usrp.get_tx_gain(0))
    if not np.isclose(actual_rate, requested_rate, atol=1.0, rtol=0):
        raise RuntimeError(f"UHD coerced TX rate to {actual_rate:g} Hz")

    args = uhd.usrp.StreamArgs("fc32", "sc16")
    args.channels = [0]
    tx = usrp.get_tx_stream(args)
    # 5000 samples is an integer number of cycles for the default 1 MHz tone
    # at 5 MS/s. The absolute sample index also preserves phase for any tone.
    chunk_samples = 5000
    total_samples = int(round(a.duration_sec*actual_rate))
    sent_total = 0
    first = True
    start = time.monotonic()
    while sent_total < total_samples:
        count = min(chunk_samples, total_samples-sent_total)
        indices = sent_total+np.arange(count, dtype=np.float64)
        samples = (0.5*np.exp(2j*np.pi*tone*indices/actual_rate)).astype(np.complex64)
        md = uhd.types.TXMetadata()
        md.has_time_spec = False
        md.start_of_burst = first
        md.end_of_burst = sent_total+count == total_samples
        sent = int(tx.send(samples, md, 1.0))
        if sent != count:
            raise RuntimeError(f"Short TX send {sent}/{count} at sample {sent_total}")
        sent_total += sent
        first = False
    elapsed = time.monotonic()-start

    async_md = uhd.types.TXAsyncMetadata()
    events = []
    deadline = time.monotonic()+1.0
    while time.monotonic() < deadline:
        if not tx.recv_async_msg(async_md, 0.1):
            continue
        code = event_value(async_md.event_code)
        events.append(EVENTS.get(code, f"unknown_{code}"))
        if code == 0x01:
            break

    print("TX-ONLY CONTINUOUS CRO SINE TEST")
    print(f"  connection:     TX/RX -> {a.attenuation_db:.1f} dB -> 50-ohm CRO")
    print(f"  TX antenna:     {usrp.get_tx_antenna(0)}")
    print(f"  sample rate:    {actual_rate/1e6:.6f} MS/s")
    print(f"  RF centre:      {actual_frequency/1e6:.6f} MHz")
    print(f"  baseband tone:  {tone/1e6:+.6f} MHz")
    print(f"  CRO carrier:    approximately {(actual_frequency+tone)/1e6:.6f} MHz")
    print(f"  TX gain:        {actual_gain:.2f} dB")
    print(f"  samples:        {sent_total}/{total_samples}")
    print(f"  send elapsed:   {elapsed:.3f} s")
    print(f"  async events:   {events}")
    failures = [e for e in events if e != "burst_ack"]
    passed = sent_total == total_samples and not failures
    print(f"Continuous CRO sine test {'PASSED' if passed else 'FAILED'}")
    return 0 if passed else 1


def main(argv=None):
    a = parse(argv)
    try:
        validate(a)
        return run(a)
    except Exception as exc:
        print(f"Continuous CRO sine test FAILED: {exc}")
        return 1


if __name__ == "__main__": sys.exit(main())
