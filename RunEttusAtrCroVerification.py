"""Vanguard X Stage 3G CRO-only ATR verification harness.

This program is deliberately independent of VanguardxMain_scheduler.py and
EttusRadarSource.py.  It exercises the B200mini ATR state machine using finite,
timed, zero-IQ TX bursts and finite, timed RX windows while the TRM and PA are
physically disconnected.

Connections (B200mini J6, high-impedance CRO probes only):
    pin 3 / GPIO_1 / FP0 bit 1 -> CRO CH1, TX ATR
    pin 4 / GPIO_2 / FP0 bit 2 -> CRO CH2, RX ATR
    pin 5 / GPIO_3 / FP0 bit 3 -> CRO CH3, overlap diagnostic
    pin 6 or pin 12 / GND       -> CRO ground

The TX/RX RF connector must be terminated in a separate 50-ohm RF load.  Do
not use a 50-ohm CRO input on a GPIO output.
"""

from __future__ import annotations

import argparse
import sys
import time
import traceback

import numpy as np

try:
    import uhd
except ImportError as exc:
    uhd = None
    _UHD_IMPORT_ERROR = exc
else:
    _UHD_IMPORT_ERROR = None


SAMPLE_RATE_HZ = 40.0e6
TX_NUM_SAMPLES = 200
TX_DURATION_SEC = TX_NUM_SAMPLES / SAMPLE_RATE_HZ
RX_START_DELAY_SEC = 6.0e-6
RX_NUM_SAMPLES = 4043
RX_DURATION_SEC = RX_NUM_SAMPLES / SAMPLE_RATE_HZ
RX_END_DELAY_SEC = RX_START_DELAY_SEC + RX_DURATION_SEC
DWELL_CADENCE_SEC = 0.100

GPIO_BANK = "FP0"
TX_ATR_BIT = 1
RX_ATR_BIT = 2
OVERLAP_BIT = 3
TX_ATR_MASK = 1 << TX_ATR_BIT
RX_ATR_MASK = 1 << RX_ATR_BIT
OVERLAP_MASK = 1 << OVERLAP_BIT
ATR_MASK = TX_ATR_MASK | RX_ATR_MASK | OVERLAP_MASK

TX_EVENT_NAMES = {
    0x01: "burst_ack",
    0x02: "underflow",
    0x04: "sequence_error",
    0x08: "time_error",
    0x10: "underflow_in_packet",
    0x20: "sequence_error_in_burst",
}


def parse_arguments(argv=None):
    parser = argparse.ArgumentParser(
        description="Run CRO-only B200mini ATR verification",
    )
    parser.add_argument("--serial", default="34A0320")
    parser.add_argument("--device-args", default="")
    parser.add_argument("--frequency-mhz", type=float, default=1000.0)
    parser.add_argument("--rx-gain-db", type=float, default=10.0)
    parser.add_argument("--prf-hz", type=float, default=2000.0)
    parser.add_argument("--pulses", type=int, default=32)
    parser.add_argument("--queue-depth", type=int, default=20)
    parser.add_argument("--lead-ms", type=float, default=5.0)
    parser.add_argument("--dwells", type=int, default=10)
    parser.add_argument(
        "--continuous",
        action="store_true",
        help="run 100 ms cadence dwells until Ctrl-C; ignores --dwells",
    )
    parser.add_argument("--timeout-sec", type=float, default=1.0)
    parser.add_argument(
        "--idle-observation-sec",
        type=float,
        default=2.0,
    )
    parser.add_argument(
        "--shutdown-observation-sec",
        type=float,
        default=0.0,
    )
    parser.add_argument(
        "--i-confirm-trm-and-pa-disconnected",
        action="store_true",
    )
    parser.add_argument(
        "--i-confirm-txrx-terminated-50-ohm",
        action="store_true",
    )
    parser.add_argument(
        "--i-confirm-cro-inputs-high-impedance",
        action="store_true",
    )
    parser.add_argument(
        "--i-understand-zero-iq-still-enables-tx-chain",
        action="store_true",
    )
    return parser.parse_args(argv)


def validate_arguments(args):
    required = (
        (
            args.i_confirm_trm_and_pa_disconnected,
            "Missing --i-confirm-trm-and-pa-disconnected",
        ),
        (
            args.i_confirm_txrx_terminated_50_ohm,
            "Missing --i-confirm-txrx-terminated-50-ohm",
        ),
        (
            args.i_confirm_cro_inputs_high_impedance,
            "Missing --i-confirm-cro-inputs-high-impedance",
        ),
        (
            args.i_understand_zero_iq_still_enables_tx_chain,
            "Missing --i-understand-zero-iq-still-enables-tx-chain",
        ),
    )
    for confirmed, message in required:
        if not confirmed:
            raise ValueError(message)

    if not 1000.0 <= args.prf_hz <= 4000.0:
        raise ValueError("PRF must be between 1000 and 4000 Hz")
    if not 1 <= args.pulses <= 128:
        raise ValueError("Pulses must be between 1 and 128")
    if not 1 <= args.queue_depth <= 20:
        raise ValueError("Queue depth must be between 1 and 20")
    if args.lead_ms < 5.0:
        raise ValueError("Command lead must be at least 5 ms")
    if not args.continuous and args.dwells <= 0:
        raise ValueError("Dwells must be positive")
    if args.timeout_sec <= 0.0:
        raise ValueError("Timeout must be positive")
    if args.idle_observation_sec < 0.0:
        raise ValueError("Idle observation time must not be negative")
    if args.shutdown_observation_sec < 0.0:
        raise ValueError("Shutdown observation time must not be negative")

    pri_sec = 1.0 / float(args.prf_hz)
    if RX_END_DELAY_SEC >= pri_sec:
        raise ValueError("The fixed CRO RX window does not fit in the PRI")
    cpi_sec = int(args.pulses) * pri_sec
    if cpi_sec + float(args.lead_ms) / 1000.0 > DWELL_CADENCE_SEC:
        raise ValueError("CPI plus command lead does not fit 100 ms cadence")


def _enum_value(value):
    try:
        return int(value.value)
    except (AttributeError, TypeError, ValueError):
        try:
            return int(value)
        except (TypeError, ValueError):
            return None


class AtrCroController:
    """Configure fail-low operational ATR outputs plus an overlap witness."""

    def __init__(self, usrp):
        self.usrp = usrp
        self.configured = False

    def _set(self, attribute, value):
        self.usrp.set_gpio_attr(
            GPIO_BANK,
            attribute,
            int(value),
            ATR_MASK,
            0,
        )

    def _read_masked(self, attribute):
        return int(self.usrp.get_gpio_attr(GPIO_BANK, attribute, 0)) & ATR_MASK

    def _require_register(self, attribute, expected):
        actual = self._read_masked(attribute)
        if actual != int(expected):
            raise RuntimeError(
                f"GPIO {attribute} readback 0x{actual:X}; "
                f"expected 0x{int(expected):X}"
            )

    def set_manual_safe_low(self):
        # Preload OUT low while the pins are still inputs, select manual GPIO,
        # then enable the output drivers.  This ordering avoids a high glitch.
        self._set("OUT", 0)
        self._set("CTRL", 0)
        self._set("DDR", ATR_MASK)
        self._require_register("CTRL", 0)
        self._require_register("DDR", ATR_MASK)
        self._require_register("OUT", 0)

    def configure_atr(self):
        available_banks = list(self.usrp.get_gpio_banks(0))
        if GPIO_BANK not in available_banks:
            raise RuntimeError(
                f"GPIO bank '{GPIO_BANK}' is unavailable; "
                f"available banks are {available_banks}"
            )

        self.set_manual_safe_low()

        # Operational outputs fail low during full duplex.  GPIO_3 is a CRO
        # witness which becomes high only if UHD enters ATR_XX.
        self._set("ATR_0X", 0)
        self._set("ATR_RX", RX_ATR_MASK)
        self._set("ATR_TX", TX_ATR_MASK)
        self._set("ATR_XX", OVERLAP_MASK)
        self._require_register("ATR_0X", 0)
        self._require_register("ATR_RX", RX_ATR_MASK)
        self._require_register("ATR_TX", TX_ATR_MASK)
        self._require_register("ATR_XX", OVERLAP_MASK)

        # CTRL is deliberately last: all state words and safe output direction
        # are established before the FPGA ATR state machine owns the pins.
        self._set("CTRL", ATR_MASK)
        self._require_register("CTRL", ATR_MASK)
        self.configured = True

    def shutdown_safe_low(self):
        self.set_manual_safe_low()
        self.configured = False


class CroAtrHarness:
    """Own the B200mini and issue bounded finite timed TX/RX pairs."""

    def __init__(self, args):
        self.args = args
        self.usrp = None
        self.rx_streamer = None
        self.tx_streamer = None
        self.atr = None
        self.tx_zeros = np.zeros(TX_NUM_SAMPLES, dtype=np.complex64)

    def initialise(self):
        if uhd is None:
            raise RuntimeError(
                "UHD Python bindings are unavailable. Confirm that "
                "'python3 -c \"import uhd\"' succeeds."
            ) from _UHD_IMPORT_ERROR

        device_args = str(self.args.device_args).strip()
        serial_arg = f"serial={self.args.serial}"
        device_args = (
            f"{serial_arg},{device_args}" if device_args else serial_arg
        )
        self.usrp = uhd.usrp.MultiUSRP(device_args)
        channel = 0
        frequency_hz = float(self.args.frequency_mhz) * 1.0e6

        self.usrp.set_rx_rate(SAMPLE_RATE_HZ, channel)
        self.usrp.set_tx_rate(SAMPLE_RATE_HZ, channel)
        self.usrp.set_rx_freq(uhd.types.TuneRequest(frequency_hz), channel)
        self.usrp.set_tx_freq(uhd.types.TuneRequest(frequency_hz), channel)
        self.usrp.set_rx_gain(float(self.args.rx_gain_db), channel)
        self.usrp.set_tx_gain(0.0, channel)
        self.usrp.set_rx_antenna("RX2", channel)
        self.usrp.set_tx_antenna("TX/RX", channel)

        actual_rx_rate = float(self.usrp.get_rx_rate(channel))
        actual_tx_rate = float(self.usrp.get_tx_rate(channel))
        if not np.isclose(actual_rx_rate, SAMPLE_RATE_HZ, atol=1.0, rtol=0.0):
            raise RuntimeError(f"UHD coerced RX rate to {actual_rx_rate:g} Hz")
        if not np.isclose(actual_tx_rate, SAMPLE_RATE_HZ, atol=1.0, rtol=0.0):
            raise RuntimeError(f"UHD coerced TX rate to {actual_tx_rate:g} Hz")
        if not np.isclose(
            float(self.usrp.get_tx_gain(channel)),
            0.0,
            atol=0.26,
            rtol=0.0,
        ):
            raise RuntimeError("UHD did not retain the fixed 0 dB TX gain")

        stream_args = uhd.usrp.StreamArgs("fc32", "sc16")
        stream_args.channels = [channel]
        self.rx_streamer = self.usrp.get_rx_stream(stream_args)
        self.tx_streamer = self.usrp.get_tx_stream(stream_args)

        self.atr = AtrCroController(self.usrp)
        self.atr.configure_atr()

    def _queue_tx(self, scheduled_time_sec):
        metadata = uhd.types.TXMetadata()
        metadata.has_time_spec = True
        metadata.time_spec = uhd.types.TimeSpec(float(scheduled_time_sec))
        metadata.start_of_burst = True
        metadata.end_of_burst = True
        sent = int(
            self.tx_streamer.send(
                self.tx_zeros,
                metadata,
                float(self.args.timeout_sec),
            )
        )
        if sent != TX_NUM_SAMPLES:
            raise RuntimeError(f"Short TX send {sent}/{TX_NUM_SAMPLES}")

    def _queue_rx(self, scheduled_time_sec):
        command = uhd.types.StreamCMD(uhd.types.StreamMode.num_done)
        command.num_samps = RX_NUM_SAMPLES
        command.stream_now = False
        command.time_spec = uhd.types.TimeSpec(float(scheduled_time_sec))
        self.rx_streamer.issue_stream_cmd(command)

    def _receive_one(self):
        metadata = uhd.types.RXMetadata()
        max_packet = int(self.rx_streamer.get_max_num_samps())
        buffer = np.zeros((1, max_packet), dtype=np.complex64)
        total = 0
        while total < RX_NUM_SAMPLES:
            request = min(max_packet, RX_NUM_SAMPLES - total)
            received = int(
                self.rx_streamer.recv(
                    buffer[:, :request],
                    metadata,
                    float(self.args.timeout_sec),
                )
            )
            error_value = _enum_value(metadata.error_code)
            if error_value not in (None, 0):
                raise RuntimeError(
                    "RX metadata error "
                    f"{metadata.strerror() if hasattr(metadata, 'strerror') else metadata.error_code}"
                )
            if received <= 0:
                raise RuntimeError("RX returned no samples without an error")
            total += received
        return total

    def _drain_tx_events_nonblocking(self):
        metadata = uhd.types.TXAsyncMetadata()
        acknowledgements = 0
        errors = []
        while self.tx_streamer.recv_async_msg(metadata, 0.0):
            code = _enum_value(metadata.event_code)
            if code == 0x01:
                acknowledgements += 1
            else:
                errors.append(TX_EVENT_NAMES.get(code, f"unknown_{code}"))
        return acknowledgements, errors

    def _wait_for_tx_events(self, expected):
        metadata = uhd.types.TXAsyncMetadata()
        acknowledgements = 0
        errors = []
        deadline = time.monotonic() + float(self.args.timeout_sec)
        while acknowledgements < expected and time.monotonic() < deadline:
            remaining = max(0.0, deadline - time.monotonic())
            if not self.tx_streamer.recv_async_msg(metadata, remaining):
                break
            code = _enum_value(metadata.event_code)
            if code == 0x01:
                acknowledgements += 1
            else:
                errors.append(TX_EVENT_NAMES.get(code, f"unknown_{code}"))
        return acknowledgements, errors

    def run_dwell(self, dwell_number):
        pri_sec = 1.0 / float(self.args.prf_hz)
        t0 = (
            float(self.usrp.get_time_now().get_real_secs())
            + float(self.args.lead_ms) / 1000.0
        )
        queue_depth = min(int(self.args.queue_depth), int(self.args.pulses))
        next_to_queue = 0
        outstanding = 0
        maximum_outstanding = 0
        acknowledgements = 0
        tx_errors = []

        def queue_pair(index):
            nonlocal outstanding, maximum_outstanding
            pulse_time = t0 + index * pri_sec
            self._queue_tx(pulse_time)
            self._queue_rx(pulse_time + RX_START_DELAY_SEC)
            outstanding += 1
            maximum_outstanding = max(maximum_outstanding, outstanding)

        while next_to_queue < queue_depth:
            queue_pair(next_to_queue)
            next_to_queue += 1

        received_samples = 0
        for _ in range(int(self.args.pulses)):
            received_samples += self._receive_one()
            outstanding -= 1
            if next_to_queue < int(self.args.pulses):
                queue_pair(next_to_queue)
                next_to_queue += 1

            # Preserve the proven pipeline ordering: first restore the timed
            # scheduling horizon, then drain any TX diagnostic events.
            new_acknowledgements, new_errors = (
                self._drain_tx_events_nonblocking()
            )
            acknowledgements += new_acknowledgements
            tx_errors.extend(new_errors)

        new_acknowledgements, new_errors = self._wait_for_tx_events(
            int(self.args.pulses) - acknowledgements
        )
        acknowledgements += new_acknowledgements
        tx_errors.extend(new_errors)
        if acknowledgements != int(self.args.pulses) or tx_errors:
            raise RuntimeError(
                f"TX ACKs {acknowledgements}/{self.args.pulses}; "
                f"events={tx_errors}"
            )
        expected_samples = int(self.args.pulses) * RX_NUM_SAMPLES
        if received_samples != expected_samples:
            raise RuntimeError(
                f"RX samples {received_samples}/{expected_samples}"
            )
        print(
            f"Dwell {dwell_number}: PASS, "
            f"TX ACKs={acknowledgements}/{self.args.pulses}, "
            f"RX samples={received_samples}/{expected_samples}, "
            f"max pairs={maximum_outstanding}"
        )

    def shutdown_safe_low(self):
        if self.atr is not None and self.usrp is not None:
            self.atr.shutdown_safe_low()

    def release(self):
        self.rx_streamer = None
        self.tx_streamer = None
        self.atr = None
        self.usrp = None


def print_plan(args):
    pri_sec = 1.0 / float(args.prf_hz)
    cpi_sec = int(args.pulses) * pri_sec
    print("VANGUARD X STAGE 3G: CRO-ONLY ATR VERIFICATION")
    print("  TRM / PA:        PHYSICALLY DISCONNECTED")
    print("  TX/RX RF port:   50-ohm RF load")
    print("  TX samples:      200 zero-IQ samples (TX chain still enabled)")
    print("  TX gain:         fixed 0 dB")
    print(f"  sample rate:     {SAMPLE_RATE_HZ / 1e6:.3f} MS/s")
    print(
        f"  PRF / PRI:       {args.prf_hz:.0f} Hz / "
        f"{pri_sec * 1e6:.3f} us"
    )
    print(
        f"  pulses / CPI:    {args.pulses} / {cpi_sec * 1e3:.3f} ms"
    )
    print(f"  dwell cadence:   {DWELL_CADENCE_SEC * 1e3:.3f} ms")
    print(f"  queue / lead:    {args.queue_depth} / {args.lead_ms:.3f} ms")
    print(
        "  run duration:    "
        + ("continuous until Ctrl-C" if args.continuous else f"{args.dwells} dwells")
    )
    print(f"  nominal TX:      0.000 to {TX_DURATION_SEC * 1e6:.3f} us")
    print(
        f"  nominal RX:      {RX_START_DELAY_SEC * 1e6:.3f} to "
        f"{RX_END_DELAY_SEC * 1e6:.3f} us "
        f"({RX_DURATION_SEC * 1e6:.3f} us)"
    )
    print("  CRO CH1:         J6 pin 3 / GPIO_1 / TX ATR")
    print("  CRO CH2:         J6 pin 4 / GPIO_2 / RX ATR")
    print("  CRO CH3:         J6 pin 5 / GPIO_3 / ATR_XX witness")
    print("  CRO ground:      J6 pin 6 or pin 12")
    print("  CRO inputs:      high impedance; NEVER 50 ohms on GPIO")
    print(
        "  state map:       CH1/CH2/CH3: "
        "idle=000, RX=010, TX=100, overlap=001"
    )


def run(args):
    harness = CroAtrHarness(args)
    print_plan(args)
    try:
        harness.initialise()
        print("ATR register readback: PASS")
        print(
            f"Idle observation: {args.idle_observation_sec:.1f} s; "
            "all three CRO channels must be low"
        )
        time.sleep(float(args.idle_observation_sec))

        next_dwell_wall = time.monotonic()
        dwell = 1
        while args.continuous or dwell <= int(args.dwells):
            harness.run_dwell(dwell)
            next_dwell_wall += DWELL_CADENCE_SEC
            remaining = next_dwell_wall - time.monotonic()
            more_dwells = args.continuous or dwell < int(args.dwells)
            if more_dwells and remaining > 0.0:
                time.sleep(remaining)
            dwell += 1
    finally:
        if harness.usrp is not None:
            try:
                harness.shutdown_safe_low()
                print("CONTROLLED SHUTDOWN: FP0 bits 1-3 forced manual low")
                print(
                    f"Shutdown observation: "
                    f"{args.shutdown_observation_sec:.1f} s; "
                    "all three CRO channels must remain low"
                )
                time.sleep(float(args.shutdown_observation_sec))
            except Exception:
                print("WARNING: failed to establish/read back safe GPIO low")
                traceback.print_exc()
            finally:
                harness.release()

    print("Stage 3G transport completed; enter CRO measurements in the worksheet")
    print(
        "After process/device release the documented B200mini GPIO state is "
        "high-Z with internal pull-ups; this is not an accepted TRM interface."
    )
    return 0


def main(argv=None):
    try:
        args = parse_arguments(argv)
        validate_arguments(args)
        return run(args)
    except KeyboardInterrupt:
        print("Stage 3G interrupted by operator; safe-low shutdown attempted")
        return 130
    except Exception:
        print("Stage 3G CRO-only ATR verification FAILED")
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())
