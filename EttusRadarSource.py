"""
Vanguard X - EttusRadarSource.py

Stage 3F guarded bounded timed-TX/RX source for an Ettus B200mini.

The setup now runs automatically during Initialise() and configures:

J6 pin 3, GPIO_1, as TX ATR
J6 pin 4, GPIO_2, as RX ATR
ATR_0X: both low
ATR_RX: RX high, TX low
ATR_TX: TX high, RX low
J6 pin 5, GPIO_3, as the ATR_XX overlap witness
ATR_XX: TX and RX low; overlap witness high

Key behaviour:
- The hardware clock is read once to establish the absolute dwell start.
- A bounded queue holds at most 20 individual finite PRI pairs.
- One future pair is appended after each collected RX window.
- Every TX hook and RX command remains finite and pulse-specific.
- No dwell-length command queue and no continuous RX are used.
- Returned IQ shape is NumPulses x NumSamples.
- Receive-only remains the default and creates no TX streamer.
- Stage 3E1 timed TX/RX is available only behind explicit RF-output,
  attenuated-loopback, and minimum-attenuation safety acknowledgements.
- Each enabled pulse is one finite timed TX burst paired with one finite RX
  window on the same fixed absolute PRI grid.
- In the RF target-emulator profile, TargetScenario selects the strongest
  in-gate parent object each dwell. The main pulse is moved (never copied) by
  that object's calibrated range delay while boresight is within +/-2 deg;
  its live radial velocity supplies the pulse-to-pulse Doppler phase.
- Matched filtering and all downstream processing remain after the full CPI.
- TX asynchronous events are drained only after replenishing the bounded
  scheduling horizon; remaining burst acknowledgements are collected after
  the complete CPI capture.
- ATR GPIO is disabled by default.  The Stage 3H guarded loopback profile may
  enable the CRO-verified mapping after explicit physical acknowledgements.
- The current timed-TX profiles prepend eight zero samples (200 ns at
  40 MS/s) to the transport burst without changing the waveform-library or
  matched-filter reference.  Delayed Stage 3F targets retain the RF-relative
  range origin after this padding.
- Optional software IQ injection can add synthetic targets to each received PRI.
"""

from __future__ import annotations

import time
from typing import Callable, Optional

import numpy as np

from DataTypes import RawDwellData
from RfScenarioTarget import SelectStrongestScenarioTarget

try:
    import uhd
except ImportError as exc:
    uhd = None
    _UHD_IMPORT_ERROR = exc
else:
    _UHD_IMPORT_ERROR = None


IqInjector = Callable[[np.ndarray, int, object, dict], np.ndarray]


TX_EVENT_NAMES = {
    0x01: "burst_ack",
    0x02: "underflow",
    0x04: "sequence_error",
    0x08: "time_error",
    0x10: "underflow_in_packet",
    0x20: "sequence_error_in_burst",
}


class EttusRadarSource:
    """Guarded fixed-depth individual-PRI TX/RX pipeline."""

    def __init__(
        self,
        Config,
        TheWaveformLibrary=None,
        iq_injector: Optional[IqInjector] = None,
    ):
        self.Config = Config
        self.TheWaveformLibrary = TheWaveformLibrary
        self.IqInjector = iq_injector

        self.OperatingMode = str(
            Config.get("EttusOperatingMode", "RECEIVE_ONLY")
        ).strip().upper()
        self.TimedTransmitEnabled = bool(
            Config.get("EttusTimedTransmitEnabled", False)
        )
        if self.OperatingMode not in ("RECEIVE_ONLY", "TIMED_TX_RX"):
            raise ValueError(
                "EttusOperatingMode must be RECEIVE_ONLY or TIMED_TX_RX"
            )
        if (
            self.OperatingMode == "RECEIVE_ONLY"
            and self.TimedTransmitEnabled
        ):
            raise ValueError(
                "RECEIVE_ONLY requires EttusTimedTransmitEnabled=False"
            )
        if (
            self.OperatingMode == "TIMED_TX_RX"
            and not self.TimedTransmitEnabled
        ):
            raise ValueError(
                "TIMED_TX_RX requires EttusTimedTransmitEnabled=True"
            )

        self.RfOutputAcknowledged = bool(
            Config.get("EttusRfOutputAcknowledged", False)
        )
        self.LoopbackConfirmed = bool(
            Config.get("EttusLoopbackConfirmed", False)
        )
        self.ExternalAttenuationDb = float(
            Config.get("EttusExternalAttenuationDb", 0.0)
        )
        self.MinimumLoopbackAttenuationDb = float(
            Config.get("EttusMinimumLoopbackAttenuationDb", 30.0)
        )
        if self.TimedTransmitEnabled:
            if not self.RfOutputAcknowledged:
                raise RuntimeError(
                    "Timed TX/RX requires explicit RF-output acknowledgement"
                )
            if not self.LoopbackConfirmed:
                raise RuntimeError(
                    "Stage 3E1 timed TX/RX requires confirmation of the "
                    "TX/RX-to-attenuator-to-RX2 loopback"
                )
            if (
                self.ExternalAttenuationDb
                < self.MinimumLoopbackAttenuationDb
            ):
                raise RuntimeError(
                    "Stage 3E1 timed TX/RX requires at least "
                    f"{self.MinimumLoopbackAttenuationDb:.1f} dB "
                    "external attenuation"
                )

        self.Usrp = None
        self.RxStreamer = None
        self.TxStreamer = None
        self.Channel = int(Config.get("EttusRxChannel", 0))
        self.TxChannel = int(Config.get("EttusTxChannel", self.Channel))
        self.CpuFormat = str(Config.get("EttusCpuFormat", "fc32"))
        self.WireFormat = str(Config.get("EttusWireFormat", "sc16"))
        self.ReceiveTimeoutSec = float(
            Config.get("EttusReceiveTimeoutSec", 1.0)
        )
        self.TxSendTimeoutSec = float(
            Config.get("EttusTxSendTimeoutSec", self.ReceiveTimeoutSec)
        )
        self.TxAsyncTimeoutSec = float(
            Config.get("EttusTxAsyncTimeoutSec", self.ReceiveTimeoutSec)
        )
        self.TxAmplitudeScale = float(
            Config.get("EttusTxAmplitudeScale", 1.0)
        )
        self.MaximumStage3E1TxGainDb = float(
            Config.get("EttusMaximumStage3E1TxGainDb", 50.0)
        )
        if not 0.0 < self.TxAmplitudeScale <= 1.0:
            raise ValueError("EttusTxAmplitudeScale must be in (0, 1]")

        self.RfTargetEmulatorEnabled = bool(
            Config.get("EttusRfTargetEmulatorEnabled", False)
        )
        self.RfTargetUseScenario = bool(
            Config.get("EttusRfTargetUseScenario", False)
        )
        self.RfTargetRangeM = float(
            Config.get("EttusRfTargetRangeM", 6000.0)
        )
        self.RfTargetBearingDeg = float(
            Config.get("EttusRfTargetBearingDeg", 80.0)
        )
        self.RfTargetAngleHalfWidthDeg = float(
            Config.get("EttusRfTargetAngleHalfWidthDeg", 2.0)
        )
        self.RfTargetRadialVelocityMps = float(
            Config.get("EttusRfTargetRadialVelocityMps", 0.0)
        )
        self.LoopbackHardwareDelaySamples = int(
            Config.get("EttusLoopbackHardwareDelaySamples", 166)
        )
        self.RfTargetAmplitudeScale = float(
            Config.get("EttusRfTargetAmplitudeScale", 1.0)
        )
        if self.RfTargetEmulatorEnabled:
            if not self.TimedTransmitEnabled:
                raise ValueError(
                    "RF target emulation requires timed TX/RX"
                )
            if self.RfTargetRangeM <= 0.0:
                raise ValueError("RF target range must be positive")
            if not np.isclose(
                self.RfTargetAngleHalfWidthDeg,
                2.0,
                rtol=0.0,
                atol=1.0e-12,
            ):
                raise ValueError(
                    "Stage 3F target bearing gate is fixed at theta +/-2 deg"
                )
            if self.LoopbackHardwareDelaySamples < 0:
                raise ValueError(
                    "Loopback hardware delay samples must not be negative"
                )
            if not 0.0 < self.RfTargetAmplitudeScale <= 1.0:
                raise ValueError(
                    "RF target amplitude scale must be in (0, 1]"
                )
        elif self.RfTargetUseScenario:
            raise ValueError(
                "Scenario RF targets require RF target emulation"
            )
        self.CommandLeadTimeSec = float(
            Config.get("EttusCommandLeadTimeSec", 0.005)
        )
        if self.CommandLeadTimeSec <= 0.0:
            raise ValueError("EttusCommandLeadTimeSec must be positive")
        self.Debug = bool(Config.get("EttusDebug", False))
        self.CommandQueueDepth = int(
            Config.get("EttusCommandQueueDepth", 20)
        )
        self.RxWarmupEnabled = bool(
            Config.get("EttusRxWarmupEnabled", True)
        )
        if not 1 <= self.CommandQueueDepth <= 32:
            raise ValueError(
                "EttusCommandQueueDepth must be between 1 and 32"
            )
        if self.TimedTransmitEnabled and self.CommandQueueDepth > 20:
            raise ValueError(
                "Stage 3E1 timed TX/RX queue depth must not exceed 20"
            )

        # B200mini front-panel GPIO/ATR configuration.  The configuration
        # values are logical GPIO bit numbers, not physical connector pins:
        # J6 pin 3 -> GPIO_1 -> bit 1 (TX ATR)
        # J6 pin 4 -> GPIO_2 -> bit 2 (RX ATR)
        self.AtrGpioEnabled = bool(
            Config.get("EttusAtrGpioEnabled", False)
        )
        self.AtrCroVerifiedAcknowledged = bool(
            Config.get("EttusAtrCroVerifiedAcknowledged", False)
        )
        self.TrmPaDisconnectedConfirmed = bool(
            Config.get("EttusTrmPaDisconnectedConfirmed", False)
        )
        self.AtrAllowOverlapForSimulation = bool(
            Config.get("EttusAtrAllowOverlapForSimulation", False)
        )
        self.TxLeadingZeroSamples = int(
            Config.get("EttusTxLeadingZeroSamples", 0)
        )
        if self.TxLeadingZeroSamples < 0:
            raise ValueError(
                "EttusTxLeadingZeroSamples must not be negative"
            )
        self.GpioBank = str(Config.get("EttusGPIOBank", "FP0"))
        self.TxAtrGpioBit = int(Config.get("EttusTxAtrGPIO", 1))
        self.RxAtrGpioBit = int(Config.get("EttusRxAtrGPIO", 2))
        self.OverlapAtrGpioBit = int(
            Config.get("EttusAtrOverlapGPIO", 3)
        )
        if min(
            self.TxAtrGpioBit,
            self.RxAtrGpioBit,
            self.OverlapAtrGpioBit,
        ) < 0:
            raise ValueError("ATR GPIO bit numbers must be non-negative")
        self.TxAtrMask = 1 << self.TxAtrGpioBit
        self.RxAtrMask = 1 << self.RxAtrGpioBit
        self.OverlapAtrMask = 1 << self.OverlapAtrGpioBit
        self.AtrGpioMask = (
            self.TxAtrMask | self.RxAtrMask | self.OverlapAtrMask
        )
        if self.AtrGpioEnabled:
            if not self.TimedTransmitEnabled:
                raise RuntimeError("ATR GPIO requires timed TX/RX mode")
            if not self.AtrCroVerifiedAcknowledged:
                raise RuntimeError(
                    "ATR GPIO requires explicit CRO-verification acknowledgement"
                )
            if not self.TrmPaDisconnectedConfirmed:
                raise RuntimeError(
                    "Stage 3H ATR loopback requires TRM and PA disconnected"
                )
            if (
                self.GpioBank != "FP0"
                or self.TxAtrGpioBit != 1
                or self.RxAtrGpioBit != 2
                or self.OverlapAtrGpioBit != 3
            ):
                raise RuntimeError(
                    "Stage 3H requires verified FP0 GPIO bits 1=TX, "
                    "2=RX and 3=ATR_XX witness"
                )
            if self.TxLeadingZeroSamples != 8:
                raise RuntimeError(
                    "Stage 3H requires exactly eight leading zero samples"
                )
        if self.AtrAllowOverlapForSimulation:
            if not self.AtrGpioEnabled:
                raise RuntimeError(
                    "Simulation ATR overlap requires ATR GPIO enabled"
                )
            if not self.RfTargetEmulatorEnabled:
                raise RuntimeError(
                    "Simulation ATR overlap is restricted to RF target emulation"
                )
        if (
            self.AtrGpioEnabled
            and self.RfTargetEmulatorEnabled
            and not self.AtrAllowOverlapForSimulation
        ):
            raise RuntimeError(
                "ATR plus delayed RF target emulation requires the explicit "
                "simulation-overlap profile"
            )
        self._configured_sample_rate = None
        self._receive_path_warmed = False
        self._initialised = False
        self._atr_configured = False

    def Initialise(self):
        if uhd is None:
            raise RuntimeError(
                "UHD Python bindings are unavailable. Confirm that "
                "'python3 -c \"import uhd\"' succeeds."
            ) from _UHD_IMPORT_ERROR

        if self._initialised:
            return

        args = self._build_device_args()
        self.Usrp = uhd.usrp.MultiUSRP(args)

        frequency_hz = float(
            self.Config.get("EttusRxFrequencyHz", 1.0e9)
        )
        gain_db = float(self.Config.get("EttusRxGainDb", 10.0))
        antenna = str(self.Config.get("EttusRxAntenna", "RX2"))

        self.Usrp.set_rx_freq(
            uhd.types.TuneRequest(frequency_hz),
            self.Channel,
        )
        self.Usrp.set_rx_gain(gain_db, self.Channel)
        self.Usrp.set_rx_antenna(antenna, self.Channel)

        tx_frequency_hz = float(
            self.Config.get("EttusTxFrequencyHz", frequency_hz)
        )
        tx_gain_db = float(self.Config.get("EttusTxGainDb", 0.0))
        tx_antenna = str(
            self.Config.get("EttusTxAntenna", "TX/RX")
        )
        if not 0.0 <= tx_gain_db <= self.MaximumStage3E1TxGainDb:
            raise ValueError(
                "EttusTxGainDb must be between 0 and "
                f"{self.MaximumStage3E1TxGainDb:.1f} dB in Stage 3E1"
            )

        stream_args = uhd.usrp.StreamArgs(
            self.CpuFormat,
            self.WireFormat,
        )
        stream_args.channels = [self.Channel]
        self.RxStreamer = self.Usrp.get_rx_stream(stream_args)

        if self.TimedTransmitEnabled:
            initial_tx_rate_hz = float(
                self.Config.get("EttusSampleRateHz", 40.0e6)
            )
            self.Usrp.set_tx_rate(
                initial_tx_rate_hz,
                self.TxChannel,
            )
            self.Usrp.set_tx_freq(
                uhd.types.TuneRequest(tx_frequency_hz),
                self.TxChannel,
            )
            self.Usrp.set_tx_gain(tx_gain_db, self.TxChannel)
            self.Usrp.set_tx_antenna(tx_antenna, self.TxChannel)

            actual_tx_gain_db = float(
                self.Usrp.get_tx_gain(self.TxChannel)
            )
            if not np.isclose(
                actual_tx_gain_db,
                tx_gain_db,
                rtol=0.0,
                atol=0.26,
            ):
                raise RuntimeError(
                    f"UHD coerced TX gain to {actual_tx_gain_db:.2f} dB; "
                    f"requested {tx_gain_db:.2f} dB"
                )

            tx_stream_args = uhd.usrp.StreamArgs(
                self.CpuFormat,
                self.WireFormat,
            )
            tx_stream_args.channels = [self.TxChannel]
            self.TxStreamer = self.Usrp.get_tx_stream(tx_stream_args)

        if self.AtrGpioEnabled:
            self._configure_atr_gpio()

        self._initialised = True
        print(
            "Ettus source initialised: "
            f"device='{args or 'auto'}', "
            f"RX={self.Usrp.get_rx_freq(self.Channel):.3f} Hz, "
            f"gain={self.Usrp.get_rx_gain(self.Channel):.2f} dB, "
            f"antenna={self.Usrp.get_rx_antenna(self.Channel)}, "
            f"mode={self.OperatingMode}, "
            f"timed TX={'enabled' if self.TimedTransmitEnabled else 'disabled'}, "
            f"ATR GPIO={'enabled' if self.AtrGpioEnabled else 'disabled'}"
        )
        if self.TimedTransmitEnabled:
            print(
                "Ettus Stage 3E1 TX configured: "
                f"TX={self.Usrp.get_tx_freq(self.TxChannel):.3f} Hz, "
                f"gain={self.Usrp.get_tx_gain(self.TxChannel):.2f} dB, "
                f"antenna={self.Usrp.get_tx_antenna(self.TxChannel)}, "
                f"external attenuation={self.ExternalAttenuationDb:.1f} dB"
            )
        if self.RfTargetEmulatorEnabled:
            if self.RfTargetUseScenario:
                print(
                    "Ettus Stage 3F RF target emulator: "
                    "TargetScenario strongest in-gate parent, "
                    "true bearing +/-2.00 deg, one delayed pulse per PRI"
                )
            else:
                target_offset_us = 1.0e6 * self._target_tx_offset_sec(
                    float(self.Config.get("EttusSampleRateHz", 40.0e6))
                )
                print(
                    "Ettus Stage 3F RF target emulator: "
                    f"range={self.RfTargetRangeM / 1000.0:.3f} km, "
                    f"bearing={self.RfTargetBearingDeg:.2f} +/-2.00 deg, "
                    f"velocity={self.RfTargetRadialVelocityMps:.2f} m/s, "
                    f"TX offset={target_offset_us:.3f} us"
                )

    def Shutdown(self):
        if self.AtrGpioEnabled and self.Usrp is not None:
            try:
                self._force_atr_safe_low()
            except Exception as exc:
                print(
                    "WARNING: ATR safe-low shutdown/readback failed: "
                    f"{exc}"
                )

        if self.RxStreamer is not None and uhd is not None:
            try:
                command = uhd.types.StreamCMD(
                    uhd.types.StreamMode.stop_cont
                )
                self.RxStreamer.issue_stream_cmd(command)
            except Exception:
                pass

        self.RxStreamer = None
        self.TxStreamer = None
        self.Usrp = None
        self._configured_sample_rate = None
        self._receive_path_warmed = False
        self._initialised = False
        self._atr_configured = False
        print("Ettus source shutdown")

    def SetIqInjector(self, iq_injector: Optional[IqInjector]):
        """Set or clear the optional synthetic-target injection callback."""
        self.IqInjector = iq_injector

    def ExecuteDwell(self, ThisDwell):
        if not self._initialised:
            raise RuntimeError(
                "Call EttusRadarSource.Initialise() before ExecuteDwell()."
            )

        # RadarExecutor has already validated the waveform/timing solution.
        # Consume the dwell value here so the UHD rate cannot diverge from it.
        sample_rate = float(ThisDwell.SampleRate)
        num_samples = int(ThisDwell.NumSamples)
        num_pulses = self._get_num_pulses(ThisDwell)

        if sample_rate <= 0.0:
            raise ValueError("SampleRate must be positive")
        if num_samples <= 0:
            raise ValueError("NumSamples must be positive")
        if num_pulses <= 0:
            raise ValueError("NumPulses must be positive")

        actual_sample_rate = self._configure_sample_rate(sample_rate)

        warmup_performed = False
        warmup_diagnostics = None
        if self.RxWarmupEnabled and not self._receive_path_warmed:
            warmup_diagnostics = self._warm_up_receive_path(num_samples)
            self._receive_path_warmed = True
            warmup_performed = True

        pulse_pri_sec = np.asarray(
            [
                self._get_pulse_pri_sec(ThisDwell, i)
                for i in range(num_pulses)
            ],
            dtype=np.float64,
        )

        pulse_times_sec = np.zeros(num_pulses, dtype=np.float64)
        if num_pulses > 1:
            pulse_times_sec[1:] = np.cumsum(pulse_pri_sec[:-1])

        pulse_waveform_ids = [
            self._get_pulse_waveform_id(ThisDwell, i)
            for i in range(num_pulses)
        ]
        pulse_rx_start_delay_sec = np.asarray(
            [
                self._get_pulse_rx_start_delay_sec(ThisDwell, i)
                for i in range(num_pulses)
            ],
            dtype=np.float64,
        )
        if np.any(pulse_rx_start_delay_sec < 0.0):
            raise ValueError("RX start delay must not be negative")
        tx_leading_zero_duration_sec = (
            self.TxLeadingZeroSamples / float(actual_sample_rate)
        )
        pulse_range_reference_rx_start_delay_sec = (
            pulse_rx_start_delay_sec - tx_leading_zero_duration_sec
        )
        if np.any(pulse_range_reference_rx_start_delay_sec < 0.0):
            raise ValueError(
                "RX start precedes the RF waveform range-time origin"
            )

        pulse_num_rx_samples = np.asarray(
            [
                self._get_pulse_num_rx_samples(ThisDwell, i, num_samples)
                for i in range(num_pulses)
            ],
            dtype=np.int64,
        )
        if np.any(pulse_num_rx_samples != num_samples):
            raise ValueError(
                "EttusRadarSource currently requires the same RX sample "
                "count on every pulse in a dwell"
            )

        rf_target = self._resolve_rf_target_for_dwell(
            ThisDwell=ThisDwell,
            sample_rate_hz=actual_sample_rate,
            num_samples=num_samples,
            waveform_id=pulse_waveform_ids[0],
            rx_start_delay_sec=float(
                pulse_range_reference_rx_start_delay_sec[0]
            ),
        )
        rf_target_active = bool(rf_target["Active"])
        rf_target_angle_error_deg = float(rf_target["AngleErrorDeg"])
        rf_target_tx_offset_sec = 0.0
        if rf_target_active:
            rf_target_tx_offset_sec = self._target_tx_offset_sec(
                actual_sample_rate,
                target_range_m=rf_target["RangeM"],
            )
        pulse_tx_offset_sec = np.full(
            num_pulses,
            rf_target_tx_offset_sec,
            dtype=np.float64,
        )

        iq = np.zeros(
            (num_pulses, num_samples),
            dtype=np.complex64,
        )
        pulse_valid = np.ones(num_pulses, dtype=bool)
        pulse_diagnostics = []

        # Match the proven S-band implementation: read device time once to
        # establish T0, then derive every PRI from the fixed absolute grid.
        # Per-PRI get_time_now() calls add a slow USB control transaction and
        # can themselves make the next command late.
        hardware_now_sec = self.Usrp.get_time_now().get_real_secs()
        hardware_time_query_count = 1
        hardware_start_sec = hardware_now_sec + self.CommandLeadTimeSec
        wall_start = time.time()

        # CommandLeadTimeSec is applied once. Subsequent pulse times are always
        # T0 + n*PRI; they are never retimed from host execution time.
        scheduled_pri_times_sec = hardware_start_sec + pulse_times_sec
        scheduled_rx_times_sec = (
            scheduled_pri_times_sec + pulse_rx_start_delay_sec
        )
        if np.any(np.diff(scheduled_rx_times_sec) < 0.0):
            raise ValueError("RX windows must be scheduled in time order")

        minimum_pri_sec = float(np.min(pulse_pri_sec))
        queue_depth = min(self.CommandQueueDepth, num_pulses)

        maximum_outstanding_receive_commands = 0
        outstanding_receive_commands = 0
        outstanding_after_issue = np.zeros(num_pulses, dtype=np.int64)
        transmit_queued_by_pulse = np.zeros(num_pulses, dtype=bool)
        transmit_command_count = 0
        transmit_acknowledgement_count = 0
        transmit_event_errors = []

        def queue_pulse(queue_index):
            nonlocal outstanding_receive_commands
            nonlocal maximum_outstanding_receive_commands
            nonlocal transmit_command_count

            if outstanding_receive_commands >= queue_depth:
                raise RuntimeError(
                    "Individual PRI pipeline exceeded its bounded horizon"
                )
            scheduled_pri_time_sec = float(
                scheduled_pri_times_sec[queue_index]
            )
            scheduled_time_sec = float(
                scheduled_rx_times_sec[queue_index]
            )

            # Individual-pair ordering inherited from S-band:
            #   1. queue finite timed TX at Tn (disabled here);
            #   2. arm finite RX at Tn + RxStartDelay.
            transmit_queued = self._queue_transmit_for_pri(
                ThisDwell=ThisDwell,
                pulse_index=queue_index,
                scheduled_pri_time_sec=scheduled_pri_time_sec,
                tx_time_offset_sec=float(
                    pulse_tx_offset_sec[queue_index]
                ),
                rf_target=rf_target,
            )
            if transmit_queued:
                transmit_command_count += 1
            self._issue_receive_command(
                num_samples=num_samples,
                scheduled_time_sec=scheduled_time_sec,
            )
            outstanding_receive_commands += 1
            maximum_outstanding_receive_commands = max(
                maximum_outstanding_receive_commands,
                outstanding_receive_commands,
            )

            outstanding_after_issue[queue_index] = (
                outstanding_receive_commands
            )
            transmit_queued_by_pulse[queue_index] = bool(transmit_queued)

        next_queue_index = 0
        while next_queue_index < queue_depth:
            queue_pulse(next_queue_index)
            next_queue_index += 1

        for pulse_index in range(num_pulses):
            scheduled_pri_time_sec = float(
                scheduled_pri_times_sec[pulse_index]
            )
            scheduled_time_sec = float(
                scheduled_rx_times_sec[pulse_index]
            )
            try:
                pulse_iq, diag = self._receive_scheduled_pri(
                    num_samples=num_samples,
                )
            finally:
                outstanding_receive_commands -= 1

            # Restore the time-based horizon immediately after collection,
            # before IQ copying or diagnostic formatting consumes host time.
            if (
                next_queue_index < num_pulses
                and outstanding_receive_commands < queue_depth
            ):
                queue_pulse(next_queue_index)
                next_queue_index += 1

            # Replenish the scheduling horizon before touching TX diagnostic
            # events. This ordering was proven by the Stage 3D loopback test.
            if self.TimedTransmitEnabled:
                acknowledged, errors = (
                    self._drain_tx_events_nonblocking()
                )
                transmit_acknowledgement_count += acknowledged
                transmit_event_errors.extend(errors)

            copied = min(len(pulse_iq), num_samples)
            if copied:
                iq[pulse_index, :copied] = pulse_iq[:copied]
            if copied != num_samples:
                pulse_valid[pulse_index] = False

            context = {
                "ScheduledPriHardwareTimeSec": scheduled_pri_time_sec,
                "ScheduledHardwareTimeSec": float(scheduled_time_sec),
                "PulseTimeSec": float(pulse_times_sec[pulse_index]),
                "PriSec": float(pulse_pri_sec[pulse_index]),
                "RxStartDelaySec": float(
                    pulse_range_reference_rx_start_delay_sec[pulse_index]
                ),
                "ScheduledRxStartDelaySec": float(
                    pulse_rx_start_delay_sec[pulse_index]
                ),
                "RfPulseStartDelaySec": float(
                    tx_leading_zero_duration_sec
                ),
                "WaveformId": str(pulse_waveform_ids[pulse_index]),
                "ValidBeforeInjection": bool(pulse_valid[pulse_index]),
                "ReceiveCommandOrdinal": pulse_index + 1,
                "OutstandingReceiveCommandsAfterIssue": int(
                    outstanding_after_issue[pulse_index]
                ),
                "TransmitQueued": bool(
                    transmit_queued_by_pulse[pulse_index]
                ),
                "TransmitTimeOffsetSec": float(
                    pulse_tx_offset_sec[pulse_index]
                ),
                "RfTargetEmulatorActive": bool(rf_target_active),
                "RfTargetName": str(rf_target["Name"]),
            }

            if self.IqInjector is not None:
                injected = self.IqInjector(
                    iq[pulse_index].copy(),
                    pulse_index,
                    ThisDwell,
                    context,
                )
                injected = np.asarray(injected, dtype=np.complex64)
                if injected.shape != (num_samples,):
                    raise ValueError(
                        "IQ injector returned shape "
                        f"{injected.shape}; expected ({num_samples},)"
                    )
                iq[pulse_index] = injected
                diag["SoftwareIqInjected"] = True
            else:
                diag["SoftwareIqInjected"] = False

            diag.update(context)
            pulse_diagnostics.append(diag)

        if self.TimedTransmitEnabled:
            acknowledged, errors = self._wait_for_tx_events(
                expected_acknowledgements=(
                    transmit_command_count
                    - transmit_acknowledgement_count
                ),
                timeout_sec=self.TxAsyncTimeoutSec,
            )
            transmit_acknowledgement_count += acknowledged
            transmit_event_errors.extend(errors)

            if transmit_acknowledgement_count != transmit_command_count:
                transmit_event_errors.append(
                    "burst acknowledgements "
                    f"{transmit_acknowledgement_count}/"
                    f"{transmit_command_count}"
                )
            if transmit_event_errors:
                raise RuntimeError(
                    "Ettus timed-TX dwell failed after CPI capture: "
                    + "; ".join(transmit_event_errors)
                )

        wall_end = time.time()

        diagnostics = {
            "SourceType": "EttusRadarSource",
            "OperatingMode": str(self.OperatingMode),
            "ReceiveOnly": not self.TimedTransmitEnabled,
            "ReceiveCommandPerPri": True,
            "SBandStylePerPriLoop": True,
            "AdaptiveIndividualPriPipeline": False,
            "BoundedIndividualPriPipeline": True,
            "AllReceiveCommandsQueuedBeforeCollection": (
                queue_depth == num_pulses
            ),
            "ReceiveCommandCount": num_pulses,
            "ConfiguredCommandQueueDepth": self.CommandQueueDepth,
            "ActiveCommandQueueDepth": queue_depth,
            "CommandQueueHorizonSec": queue_depth * minimum_pri_sec,
            "MaximumOutstandingReceiveCommands": (
                maximum_outstanding_receive_commands
            ),
            "HardwareTimeQueriesPerDwell": hardware_time_query_count,
            "RxWarmupEnabled": self.RxWarmupEnabled,
            "RxWarmupPerformed": warmup_performed,
            "RxWarmupDiagnostics": warmup_diagnostics,
            "FixedAbsolutePriSchedule": True,
            "PerPriOperationOrder": (
                "QUEUE_BOUNDED_INDIVIDUAL_PAIRS; COLLECT_OLDEST; "
                "REPLENISH_ONE"
            ),
            "CommandLeadTimeAppliedOncePerDwell": True,
            "TimedTransmitEnabled": bool(self.TimedTransmitEnabled),
            "RfOutputAcknowledged": bool(self.RfOutputAcknowledged),
            "LoopbackConfirmed": bool(self.LoopbackConfirmed),
            "ExternalAttenuationDb": float(self.ExternalAttenuationDb),
            "RfTargetEmulatorEnabled": bool(
                self.RfTargetEmulatorEnabled
            ),
            "RfTargetUseScenario": bool(self.RfTargetUseScenario),
            "RfTargetEmulatorActive": bool(rf_target_active),
            "RfTargetName": str(rf_target["Name"]),
            "RfTargetRangeM": float(rf_target["RangeM"]),
            "RfTargetBearingDeg": float(rf_target["BearingDeg"]),
            "RfTargetRadialVelocityMps": float(
                rf_target["RadialVelocityMps"]
            ),
            "RfTargetScenarioEquivalentAmplitude": float(
                rf_target["EquivalentAmplitude"]
            ),
            "RfTargetScenarioConstituentReturnCount": int(
                rf_target["ConstituentReturnCount"]
            ),
            "RfTargetAngleHalfWidthDeg": float(
                self.RfTargetAngleHalfWidthDeg
            ),
            "RfTargetAngleErrorDeg": float(
                rf_target_angle_error_deg
            ),
            "RfTargetTxOffsetSec": float(rf_target_tx_offset_sec),
            "LoopbackHardwareDelaySamples": int(
                self.LoopbackHardwareDelaySamples
            ),
            "TransmitCommandCount": int(transmit_command_count),
            "TransmitBurstAcknowledgementCount": int(
                transmit_acknowledgement_count
            ),
            "TransmitEventErrors": list(transmit_event_errors),
            "MaximumOutstandingTimedPairs": int(
                maximum_outstanding_receive_commands
            ),
            "SecondTransmitPulseEnabled": False,
            "SoftwareIqInjectionEnabled": self.IqInjector is not None,
            "RequestedSampleRate": sample_rate,
            "ActualSampleRate": actual_sample_rate,
            "NumSamplesPerPulse": num_samples,
            "NumPulses": num_pulses,
            "PulseRxStartDelaySec": (
                pulse_range_reference_rx_start_delay_sec.copy()
            ),
            "PulseScheduledRxStartDelaySec": (
                pulse_rx_start_delay_sec.copy()
            ),
            "ScheduledPriHardwareTimesSec": (
                scheduled_pri_times_sec.copy()
            ),
            "ScheduledRxHardwareTimesSec": scheduled_rx_times_sec.copy(),
            "ScheduledTxHardwareTimesSec": np.where(
                transmit_queued_by_pulse,
                scheduled_pri_times_sec + pulse_tx_offset_sec,
                np.nan,
            ),
            "RxFrequencyHz": float(self.Usrp.get_rx_freq(self.Channel)),
            "RxGainDb": float(self.Usrp.get_rx_gain(self.Channel)),
            "RxAntenna": str(self.Usrp.get_rx_antenna(self.Channel)),
            "AtrGpioEnabled": bool(self.AtrGpioEnabled),
            "AtrGpioBank": str(self.GpioBank),
            "TxAtrGpioBit": int(self.TxAtrGpioBit),
            "RxAtrGpioBit": int(self.RxAtrGpioBit),
            "OverlapAtrGpioBit": int(self.OverlapAtrGpioBit),
            "AtrCroVerifiedAcknowledged": bool(
                self.AtrCroVerifiedAcknowledged
            ),
            "TrmPaDisconnectedConfirmed": bool(
                self.TrmPaDisconnectedConfirmed
            ),
            "AtrAllowOverlapForSimulation": bool(
                self.AtrAllowOverlapForSimulation
            ),
            "TxLeadingZeroSamples": int(self.TxLeadingZeroSamples),
            "TxLeadingZeroDurationSec": (
                self.TxLeadingZeroSamples / float(actual_sample_rate)
            ),
            "PulseDiagnostics": pulse_diagnostics,
            "CaptureElapsedSec": wall_end - wall_start,
        }
        if self.TimedTransmitEnabled:
            diagnostics.update({
                "TxFrequencyHz": float(
                    self.Usrp.get_tx_freq(self.TxChannel)
                ),
                "TxGainDb": float(
                    self.Usrp.get_tx_gain(self.TxChannel)
                ),
                "TxAntenna": str(
                    self.Usrp.get_tx_antenna(self.TxChannel)
                ),
            })

        return RawDwellData(
            DwellId=int(ThisDwell.DwellId),
            IQ=iq,
            SampleRate=float(actual_sample_rate),
            PRI=float(pulse_pri_sec[0]),
            TimeStamp=float(wall_end),
            PulseTimesSec=pulse_times_sec,
            PulsePriSec=pulse_pri_sec,
            PulseRxStartDelaySec=(
                pulse_range_reference_rx_start_delay_sec
            ),
            PulseWaveformIds=pulse_waveform_ids,
            PulseValid=pulse_valid,
            Diagnostics=diagnostics,
        )

    def _queue_transmit_for_pri(
        self,
        ThisDwell,
        pulse_index,
        scheduled_pri_time_sec,
        tx_time_offset_sec=None,
        rf_target=None,
    ):
        """Queue one finite waveform burst at the absolute PRI origin."""

        if not self.TimedTransmitEnabled:
            return False
        if self.OperatingMode != "TIMED_TX_RX":
            raise RuntimeError("Timed TX reached a non-transmit mode")
        if self.TxStreamer is None:
            raise RuntimeError("Timed TX is enabled but no TX streamer exists")
        if not self._get_pulse_tx_enabled(ThisDwell, pulse_index):
            return False
        if self.TheWaveformLibrary is None:
            raise RuntimeError("Timed TX requires WaveformLibrary")
        if self._configured_sample_rate is None:
            raise RuntimeError("TX sample rate has not been configured")

        waveform_id = self._get_pulse_waveform_id(
            ThisDwell,
            pulse_index,
        )
        waveform = np.asarray(
            self.TheWaveformLibrary.Get(waveform_id),
            dtype=np.complex64,
        )
        if waveform.ndim != 1 or waveform.size == 0:
            raise ValueError(
                f"Waveform {waveform_id} must be a non-empty 1D array"
            )

        pulse_plan = self._get_pulse_plan(ThisDwell, pulse_index)
        amplitude_scale = self.TxAmplitudeScale
        phase_offset_rad = 0.0
        frequency_offset_hz = 0.0
        if pulse_plan is not None:
            amplitude_scale *= float(
                getattr(pulse_plan, "AmplitudeScale", 1.0)
            )
            phase_offset_rad = float(
                getattr(pulse_plan, "PhaseOffsetRad", 0.0)
            )
            frequency_offset_hz = float(
                getattr(pulse_plan, "FrequencyOffsetHz", 0.0)
            )
        if rf_target is None:
            angle_error_deg = self._signed_angle_difference_deg(
                float(getattr(ThisDwell, "AzimuthDeg", 0.0)),
                self.RfTargetBearingDeg,
            )
            rf_target = {
                "Active": bool(
                    self.RfTargetEmulatorEnabled
                    and abs(angle_error_deg)
                    <= self.RfTargetAngleHalfWidthDeg
                ),
                "RangeM": self.RfTargetRangeM,
                "RadialVelocityMps": self.RfTargetRadialVelocityMps,
                "AmplitudeScale": self.RfTargetAmplitudeScale,
            }
        rf_target_active = bool(rf_target["Active"])
        if rf_target_active:
            amplitude_scale *= float(rf_target["AmplitudeScale"])
            wavelength_m = 299792458.0 / float(
                self.Config.get("RfFrequency", 9.4e9)
            )
            target_doppler_hz = (
                2.0 * float(rf_target["RadialVelocityMps"]) / wavelength_m
            )
            phase_offset_rad += (
                2.0
                * np.pi
                * target_doppler_hz
                * pulse_index
                * self._get_pulse_pri_sec(ThisDwell, pulse_index)
            )
        if not 0.0 < amplitude_scale <= 1.0:
            raise ValueError(
                "Combined TX amplitude scale must be in (0, 1]"
            )

        rf_waveform_samples = int(waveform.size)
        if phase_offset_rad != 0.0 or frequency_offset_hz != 0.0:
            sample_number = np.arange(waveform.size, dtype=np.float64)
            phase = (
                phase_offset_rad
                + 2.0
                * np.pi
                * frequency_offset_hz
                * sample_number
                / float(self._configured_sample_rate)
            )
            waveform = waveform * np.exp(1j * phase).astype(np.complex64)
        waveform = np.asarray(
            waveform * amplitude_scale,
            dtype=np.complex64,
        )
        if self.TxLeadingZeroSamples:
            waveform = np.concatenate((
                np.zeros(
                    self.TxLeadingZeroSamples,
                    dtype=np.complex64,
                ),
                waveform,
            ))

        rf_pulse_duration_sec = (
            rf_waveform_samples / float(self._configured_sample_rate)
        )
        tx_envelope_duration_sec = (
            waveform.size / float(self._configured_sample_rate)
        )
        pri_sec = self._get_pulse_pri_sec(ThisDwell, pulse_index)
        rx_start_delay_sec = self._get_pulse_rx_start_delay_sec(
            ThisDwell,
            pulse_index,
        )
        if tx_envelope_duration_sec > pri_sec:
            raise ValueError("TX waveform does not fit within the PRI")
        if tx_time_offset_sec is None:
            tx_time_offset_sec = (
                self._target_tx_offset_sec(
                    self._configured_sample_rate,
                    target_range_m=rf_target["RangeM"],
                )
                if rf_target_active
                else 0.0
            )
        tx_time_offset_sec = float(tx_time_offset_sec)
        if tx_time_offset_sec < 0.0:
            raise ValueError("TX time offset must not be negative")

        if rf_target_active:
            desired_echo_delay_sec = (
                2.0 * float(rf_target["RangeM"]) / 299792458.0
            )
            # Dwell-plan RX timing is absolute from the transport/PRI origin,
            # while target range is measured from the first non-zero RF
            # sample.  Remove the leading-zero duration for range-window tests.
            rf_origin_delay_sec = (
                self.TxLeadingZeroSamples
                / float(self._configured_sample_rate)
            )
            range_rx_start_delay_sec = (
                rx_start_delay_sec - rf_origin_delay_sec
            )
            num_rx_samples = self._get_pulse_num_rx_samples(
                ThisDwell,
                pulse_index,
                int(getattr(ThisDwell, "NumSamples", 0)),
            )
            rx_end_delay_sec = (
                range_rx_start_delay_sec
                + num_rx_samples / float(self._configured_sample_rate)
            )
            if (
                desired_echo_delay_sec
                < range_rx_start_delay_sec - 1.0e-12
            ):
                raise ValueError(
                    "RF target echo begins before the RX window"
                )
            if (
                desired_echo_delay_sec + rf_pulse_duration_sec
                > rx_end_delay_sec + 1.0e-12
            ):
                raise ValueError(
                    "RF target echo does not fit inside the RX window"
                )
        elif tx_envelope_duration_sec > rx_start_delay_sec + 1.0e-12:
            raise ValueError(
                "Operational RX starts before the TX waveform has ended"
            )
        if tx_time_offset_sec + tx_envelope_duration_sec > pri_sec:
            raise ValueError("Delayed TX waveform does not fit within the PRI")

        metadata = uhd.types.TXMetadata()
        metadata.has_time_spec = True
        metadata.time_spec = uhd.types.TimeSpec(
            float(scheduled_pri_time_sec) + tx_time_offset_sec
        )
        metadata.start_of_burst = True
        metadata.end_of_burst = True

        sent = int(
            self.TxStreamer.send(
                waveform,
                metadata,
                self.TxSendTimeoutSec,
            )
        )
        if sent != waveform.size:
            raise RuntimeError(
                f"Pulse {pulse_index}: short TX send "
                f"{sent}/{waveform.size} samples"
            )
        return True

    def _drain_tx_events_nonblocking(self):
        """Drain available TX events without consuming scheduling horizon."""

        metadata = uhd.types.TXAsyncMetadata()
        acknowledgements = 0
        errors = []
        while self.TxStreamer.recv_async_msg(metadata, 0.0):
            code = self._error_value(metadata.event_code)
            if code == 0x01:
                acknowledgements += 1
            else:
                errors.append(
                    f"TX async event {TX_EVENT_NAMES.get(code, f'unknown_{code}')}"
                )
        return acknowledgements, errors

    def _wait_for_tx_events(self, expected_acknowledgements, timeout_sec):
        """Collect remaining burst ACKs once the complete CPI is captured."""

        expected_acknowledgements = int(expected_acknowledgements)
        if expected_acknowledgements <= 0:
            return 0, []

        metadata = uhd.types.TXAsyncMetadata()
        acknowledgements = 0
        errors = []
        deadline = time.monotonic() + float(timeout_sec)
        while (
            acknowledgements < expected_acknowledgements
            and time.monotonic() < deadline
        ):
            remaining = max(0.0, deadline - time.monotonic())
            if not self.TxStreamer.recv_async_msg(metadata, remaining):
                break
            code = self._error_value(metadata.event_code)
            if code == 0x01:
                acknowledgements += 1
            else:
                errors.append(
                    f"TX async event {TX_EVENT_NAMES.get(code, f'unknown_{code}')}"
                )
        return acknowledgements, errors

    def _configure_atr_gpio(self):
        """Configure the CRO-verified fail-low Stage 3H ATR mapping."""
        available_banks = list(self.Usrp.get_gpio_banks(0))
        if self.GpioBank not in available_banks:
            raise RuntimeError(
                f"GPIO bank '{self.GpioBank}' is unavailable; "
                f"available banks are {available_banks}"
            )

        if len({
            self.TxAtrGpioBit,
            self.RxAtrGpioBit,
            self.OverlapAtrGpioBit,
        }) != 3:
            raise ValueError(
                "TX, RX and ATR_XX witness GPIOs must use different bits"
            )
        if min(
            self.TxAtrGpioBit,
            self.RxAtrGpioBit,
            self.OverlapAtrGpioBit,
        ) < 0:
            raise ValueError(
                "ATR GPIO bit numbers must be non-negative"
            )

        try:
            # Begin in manual safe-low.  OUT is preloaded while the pins are
            # inputs, manual control is selected, and only then are the output
            # drivers enabled.  This is the ordering proven by the CRO harness.
            self._force_atr_safe_low()

            # Normal operation fails both operational outputs low during full
            # duplex.  The explicit simulation-only target profile instead
            # drives TX and RX high together; GPIO_3 remains the witness.
            atr_xx_value = self.OverlapAtrMask
            if self.AtrAllowOverlapForSimulation:
                atr_xx_value |= self.TxAtrMask | self.RxAtrMask
            self._set_atr_gpio_attr("ATR_0X", 0)
            self._set_atr_gpio_attr("ATR_RX", self.RxAtrMask)
            self._set_atr_gpio_attr("ATR_TX", self.TxAtrMask)
            self._set_atr_gpio_attr("ATR_XX", atr_xx_value)
            self._require_atr_gpio_attr("ATR_0X", 0)
            self._require_atr_gpio_attr("ATR_RX", self.RxAtrMask)
            self._require_atr_gpio_attr("ATR_TX", self.TxAtrMask)
            self._require_atr_gpio_attr("ATR_XX", atr_xx_value)

            # Hand ownership to the FPGA only after all state words and the
            # safe output direction have been established and read back.
            self._set_atr_gpio_attr("CTRL", self.AtrGpioMask)
            self._require_atr_gpio_attr("CTRL", self.AtrGpioMask)
            self._atr_configured = True
        except Exception:
            try:
                self._force_atr_safe_low()
            except Exception:
                pass
            raise

        print(
            "Ettus ATR GPIO configured: "
            f"bank={self.GpioBank}, "
            f"J6 pin 3=TX (GPIO_{self.TxAtrGpioBit}), "
            f"J6 pin 4=RX (GPIO_{self.RxAtrGpioBit}), "
            f"J6 pin 5=ATR_XX witness (GPIO_{self.OverlapAtrGpioBit}), "
            "ATR_XX="
            f"{'TX+RX+WITNESS' if self.AtrAllowOverlapForSimulation else 'WITNESS_ONLY'}"
        )

    def _set_atr_gpio_attr(self, attribute, value):
        self.Usrp.set_gpio_attr(
            self.GpioBank,
            str(attribute),
            int(value),
            self.AtrGpioMask,
            0,
        )

    def _read_atr_gpio_attr(self, attribute):
        return int(
            self.Usrp.get_gpio_attr(
                self.GpioBank,
                str(attribute),
                0,
            )
        ) & self.AtrGpioMask

    def _require_atr_gpio_attr(self, attribute, expected):
        actual = self._read_atr_gpio_attr(attribute)
        expected = int(expected) & self.AtrGpioMask
        if actual != expected:
            raise RuntimeError(
                f"GPIO {attribute} readback 0x{actual:X}; "
                f"expected 0x{expected:X}"
            )

    def _force_atr_safe_low(self):
        """Take manual GPIO ownership and drive all ATR pins low."""
        self._set_atr_gpio_attr("OUT", 0)
        self._set_atr_gpio_attr("CTRL", 0)
        self._set_atr_gpio_attr("DDR", self.AtrGpioMask)
        self._require_atr_gpio_attr("CTRL", 0)
        self._require_atr_gpio_attr("DDR", self.AtrGpioMask)
        self._require_atr_gpio_attr("OUT", 0)
        self._atr_configured = False

    def _issue_receive_command(self, num_samples, scheduled_time_sec):
        """Queue one finite timed receive window without blocking."""
        command = uhd.types.StreamCMD(
            uhd.types.StreamMode.num_done
        )
        command.num_samps = int(num_samples)
        command.stream_now = False
        command.time_spec = uhd.types.TimeSpec(
            float(scheduled_time_sec)
        )
        self.RxStreamer.issue_stream_cmd(command)

    def _warm_up_receive_path(self, num_samples):
        """Prime the RX streamer once before establishing the first dwell T0."""

        command = uhd.types.StreamCMD(uhd.types.StreamMode.num_done)
        command.num_samps = int(num_samples)
        command.stream_now = True
        self.RxStreamer.issue_stream_cmd(command)

        _, diagnostics = self._receive_scheduled_pri(
            num_samples=int(num_samples),
        )
        error_count = sum(
            int(diagnostics.get(Name, 0))
            for Name in (
                "TimeoutCount",
                "OverflowCount",
                "LateCommandCount",
                "BrokenChainCount",
                "AlignmentErrorCount",
                "BadPacketCount",
                "OtherErrorCount",
            )
        )
        if (
            int(diagnostics.get("ReceivedSamples", 0)) != int(num_samples)
            or error_count != 0
        ):
            raise RuntimeError(
                "Ettus RX warm-up failed: "
                f"{diagnostics.get('ReceivedSamples', 0)}/{num_samples} "
                "samples, metadata="
                f"{diagnostics.get('LastMetadataError', 'unknown')}"
            )
        return diagnostics

    def _receive_scheduled_pri(self, num_samples):
        """Collect samples from the next previously queued RX window."""
        metadata = uhd.types.RXMetadata()

        max_packet = int(self.RxStreamer.get_max_num_samps())
        recv_buffer = np.zeros((1, max_packet), dtype=np.complex64)
        output = np.empty(num_samples, dtype=np.complex64)

        received_total = 0
        recv_calls = 0
        timeout_count = 0
        overflow_count = 0
        late_command_count = 0
        broken_chain_count = 0
        alignment_error_count = 0
        bad_packet_count = 0
        other_error_count = 0
        last_error = "none"
        last_error_code_value = 0
        last_error_text = ""
        metadata_error_events = []
        first_sample_time_sec = None

        while received_total < num_samples:
            recv_calls += 1
            request_count = min(
                max_packet,
                num_samples - received_total,
            )

            received_now = int(
                self.RxStreamer.recv(
                    recv_buffer[:, :request_count],
                    metadata,
                    self.ReceiveTimeoutSec,
                )
            )

            error_code_value = self._error_value(metadata.error_code)
            error_name = self._error_name(metadata.error_code)
            error_text = self._metadata_error_text(metadata)

            if error_code_value not in (None, 0):
                last_error = error_name
                last_error_code_value = error_code_value
                last_error_text = error_text
                metadata_error_events.append({
                    "CodeValue": error_code_value,
                    "Name": error_name,
                    "Text": error_text,
                    "ReceivedSamplesThisCall": received_now,
                })

            if error_code_value == 0:
                if received_now > 0:
                    output[
                        received_total:received_total + received_now
                    ] = recv_buffer[0, :received_now]
                    received_total += received_now
                    if (
                        first_sample_time_sec is None
                        and getattr(metadata, "has_time_spec", False)
                    ):
                        first_sample_time_sec = float(
                            metadata.time_spec.get_real_secs()
                        )
                continue

            if error_code_value == 1:
                timeout_count += 1
                break

            if error_code_value == 8:
                overflow_count += 1
                continue

            if error_code_value == 2:
                late_command_count += 1
                break

            if error_code_value == 4:
                broken_chain_count += 1
            elif error_code_value == 12:
                alignment_error_count += 1
            elif error_code_value == 15:
                bad_packet_count += 1
            else:
                other_error_count += 1
            break

        diagnostics = {
            "RequestedSamples": int(num_samples),
            "ReceivedSamples": int(received_total),
            "ReceiveCalls": int(recv_calls),
            "TimeoutCount": int(timeout_count),
            "OverflowCount": int(overflow_count),
            "LateCommandCount": int(late_command_count),
            "BrokenChainCount": int(broken_chain_count),
            "AlignmentErrorCount": int(alignment_error_count),
            "BadPacketCount": int(bad_packet_count),
            "OtherErrorCount": int(other_error_count),
            "LastMetadataError": str(last_error),
            "LastMetadataErrorCodeValue": last_error_code_value,
            "LastMetadataErrorText": str(last_error_text),
            "MetadataErrorEvents": metadata_error_events,
            "FirstSampleHardwareTimeSec": first_sample_time_sec,
        }

        return output[:received_total].copy(), diagnostics

    def _configure_sample_rate(self, requested_rate):
        if self.TxLeadingZeroSamples and not np.isclose(
            float(requested_rate),
            40.0e6,
            rtol=0.0,
            atol=1.0,
        ):
            raise RuntimeError(
                "The eight-sample Stage 3H pre-roll requires 40 MS/s"
            )
        if self._configured_sample_rate != requested_rate:
            self._receive_path_warmed = False
            self.Usrp.set_rx_rate(float(requested_rate), self.Channel)
            actual_rx_rate = float(
                self.Usrp.get_rx_rate(self.Channel)
            )
            if self.TimedTransmitEnabled:
                self.Usrp.set_tx_rate(
                    float(requested_rate),
                    self.TxChannel,
                )
                actual_tx_rate = float(
                    self.Usrp.get_tx_rate(self.TxChannel)
                )
                if not np.isclose(
                    actual_tx_rate,
                    actual_rx_rate,
                    rtol=0.0,
                    atol=1.0,
                ):
                    raise RuntimeError(
                        f"RX/TX sample rates differ: "
                        f"{actual_rx_rate:g}/{actual_tx_rate:g} Hz"
                    )
            self._configured_sample_rate = actual_rx_rate
        return self._configured_sample_rate

    def _build_device_args(self):
        explicit = str(
            self.Config.get("EttusDeviceArgs", "")
        ).strip()
        if explicit:
            return explicit

        serial = str(self.Config.get("EttusSerial", "")).strip()
        if serial:
            return f"serial={serial}"
        return ""

    @staticmethod
    def _get_num_pulses(ThisDwell):
        if hasattr(ThisDwell, "PulsePlans"):
            return len(ThisDwell.PulsePlans)
        return int(ThisDwell.NumPulses)

    @staticmethod
    def _get_pulse_plan(ThisDwell, pulse_index):
        if hasattr(ThisDwell, "PulsePlans"):
            return ThisDwell.PulsePlans[pulse_index]
        return None

    @staticmethod
    def _get_pulse_tx_enabled(ThisDwell, pulse_index):
        pulse_plan = EttusRadarSource._get_pulse_plan(
            ThisDwell,
            pulse_index,
        )
        if pulse_plan is not None:
            return bool(pulse_plan.TxEnabled)
        return True

    @staticmethod
    def _get_pulse_waveform_id(ThisDwell, pulse_index):
        if hasattr(ThisDwell, "PulsePlans"):
            return str(ThisDwell.PulsePlans[pulse_index].WaveformId)
        return str(ThisDwell.WaveformName)

    @staticmethod
    def _get_pulse_pri_sec(ThisDwell, pulse_index):
        if hasattr(ThisDwell, "PulsePlans"):
            return float(ThisDwell.PulsePlans[pulse_index].PriSec)
        return float(ThisDwell.PRI)

    @staticmethod
    def _get_pulse_rx_start_delay_sec(ThisDwell, pulse_index):
        if hasattr(ThisDwell, "PulsePlans"):
            return float(
                ThisDwell.PulsePlans[pulse_index].RxStartDelaySec
            )
        return 0.0

    @staticmethod
    def _get_pulse_num_rx_samples(ThisDwell, pulse_index, default_value):
        if hasattr(ThisDwell, "PulsePlans"):
            value = ThisDwell.PulsePlans[pulse_index].NumRxSamples
            if value is not None:
                return int(value)
        return int(default_value)

    @staticmethod
    def _signed_angle_difference_deg(angle_deg, reference_deg):
        return (
            (float(angle_deg) - float(reference_deg) + 180.0) % 360.0
        ) - 180.0

    def _resolve_rf_target_for_dwell(
        self,
        ThisDwell,
        sample_rate_hz,
        num_samples,
        waveform_id,
        rx_start_delay_sec,
    ):
        boresight_deg = float(getattr(ThisDwell, "AzimuthDeg", 0.0))
        if self.RfTargetUseScenario:
            waveform = np.asarray(
                self.TheWaveformLibrary.Get(waveform_id)
            )
            pulse_duration_sec = waveform.size / float(sample_rate_hz)
            receive_end_sec = (
                float(rx_start_delay_sec)
                + int(num_samples) / float(sample_rate_hz)
            )
            minimum_range_m = (
                float(rx_start_delay_sec) * 299792458.0 / 2.0
            )
            maximum_range_m = (
                (receive_end_sec - pulse_duration_sec)
                * 299792458.0
                / 2.0
            )
            selected = SelectStrongestScenarioTarget(
                SceneReturns=self.Config.get("SceneReturns", []),
                BoresightDeg=boresight_deg,
                AngleHalfWidthDeg=self.RfTargetAngleHalfWidthDeg,
                MinimumRangeM=minimum_range_m,
                MaximumRangeM=maximum_range_m,
            )
            if selected is None:
                return {
                    "Active": False,
                    "Name": "",
                    "RangeM": 0.0,
                    "BearingDeg": boresight_deg,
                    "RadialVelocityMps": 0.0,
                    "AngleErrorDeg": 0.0,
                    "AmplitudeScale": self.RfTargetAmplitudeScale,
                    "EquivalentAmplitude": 0.0,
                    "ConstituentReturnCount": 0,
                }
            return {
                "Active": True,
                "Name": str(selected["Name"]),
                "RangeM": float(selected["RangeM"]),
                "BearingDeg": float(selected["BearingDeg"]),
                "RadialVelocityMps": float(
                    selected["RadialVelocityMps"]
                ),
                "AngleErrorDeg": float(selected["AngleErrorDeg"]),
                # Scenario amplitude selects the strongest parent. Absolute
                # RF amplitude remains the separately guarded loopback scale.
                "AmplitudeScale": self.RfTargetAmplitudeScale,
                "EquivalentAmplitude": float(
                    selected["EquivalentAmplitude"]
                ),
                "ConstituentReturnCount": int(
                    selected["ConstituentReturnCount"]
                ),
            }

        angle_error_deg = self._signed_angle_difference_deg(
            boresight_deg,
            self.RfTargetBearingDeg,
        )
        return {
            "Active": bool(
                self.RfTargetEmulatorEnabled
                and abs(angle_error_deg)
                <= self.RfTargetAngleHalfWidthDeg
            ),
            "Name": "FixedRfTarget",
            "RangeM": self.RfTargetRangeM,
            "BearingDeg": self.RfTargetBearingDeg,
            "RadialVelocityMps": self.RfTargetRadialVelocityMps,
            "AngleErrorDeg": angle_error_deg,
            "AmplitudeScale": self.RfTargetAmplitudeScale,
            "EquivalentAmplitude": 0.0,
            "ConstituentReturnCount": 1,
        }

    def _target_tx_offset_sec(self, sample_rate_hz, target_range_m=None):
        if target_range_m is None:
            target_range_m = self.RfTargetRangeM
        desired_echo_delay_sec = (
            2.0 * float(target_range_m) / 299792458.0
        )
        hardware_delay_sec = (
            self.LoopbackHardwareDelaySamples / float(sample_rate_hz)
        )
        offset_sec = desired_echo_delay_sec - hardware_delay_sec
        if offset_sec < 0.0:
            raise ValueError(
                "Requested RF target range is shorter than the calibrated "
                "loopback hardware delay"
            )
        return float(offset_sec)

    @staticmethod
    def _error_name(error_code):
        value = EttusRadarSource._error_value(error_code)
        names = {
            0: "none",
            1: "timeout",
            2: "late_command",
            4: "broken_chain",
            8: "overflow",
            12: "alignment",
            15: "bad_packet",
        }
        return names.get(value, str(error_code).split(".")[-1].lower())

    @staticmethod
    def _error_value(error_code):
        raw_value = getattr(error_code, "value", error_code)
        try:
            return int(raw_value)
        except (TypeError, ValueError):
            return None

    @staticmethod
    def _metadata_error_text(metadata):
        strerror = getattr(metadata, "strerror", None)
        if callable(strerror):
            try:
                return str(strerror())
            except Exception:
                return ""
        return ""
