"""
Vanguard X - EttusRadarSource.py

Stage 4A.2 receive-only source for an Ettus B200mini.

The setup now runs automatically during Initialise() and configures:

J6 pin 3, GPIO_1, as TX ATR
J6 pin 4, GPIO_2, as RX ATR
ATR_0X: both low
ATR_RX: RX high, TX low
ATR_TX: TX high, RX low
ATR_XX: both low

Key behaviour:
- One finite UHD receive command is issued for each PRI.
- Each command captures exactly NumSamples IQ samples.
- Returned IQ shape is NumPulses x NumSamples.
- No second RF pulse is transmitted.
- Optional software IQ injection can add synthetic targets to each received PRI.
"""

from __future__ import annotations

import time
from typing import Callable, Optional

import numpy as np

from DataTypes import RawDwellData

try:
    import uhd
except ImportError as exc:
    uhd = None
    _UHD_IMPORT_ERROR = exc
else:
    _UHD_IMPORT_ERROR = None


IqInjector = Callable[[np.ndarray, int, object, dict], np.ndarray]


class EttusRadarSource:
    """Receive-only Ettus source with one finite RX command per PRI."""

    def __init__(
        self,
        Config,
        TheWaveformLibrary=None,
        iq_injector: Optional[IqInjector] = None,
    ):
        self.Config = Config
        self.TheWaveformLibrary = TheWaveformLibrary
        self.IqInjector = iq_injector

        self.Usrp = None
        self.RxStreamer = None
        self.Channel = int(Config.get("EttusRxChannel", 0))
        self.CpuFormat = str(Config.get("EttusCpuFormat", "fc32"))
        self.WireFormat = str(Config.get("EttusWireFormat", "sc16"))
        self.ReceiveTimeoutSec = float(
            Config.get("EttusReceiveTimeoutSec", 1.0)
        )
        self.CommandLeadTimeSec = float(
            Config.get("EttusCommandLeadTimeSec", 0.05)
        )
        self.Debug = bool(Config.get("EttusDebug", False))

        # B200mini front-panel GPIO/ATR configuration.  The configuration
        # values are logical GPIO bit numbers, not physical connector pins:
        # J6 pin 3 -> GPIO_1 -> bit 1 (TX ATR)
        # J6 pin 4 -> GPIO_2 -> bit 2 (RX ATR)
        self.AtrGpioEnabled = bool(
            Config.get("EttusAtrGpioEnabled", True)
        )
        self.GpioBank = str(Config.get("EttusGPIOBank", "FP0"))
        self.TxAtrGpioBit = int(Config.get("EttusTxAtrGPIO", 1))
        self.RxAtrGpioBit = int(Config.get("EttusRxAtrGPIO", 2))
        self.TxAtrMask = 1 << self.TxAtrGpioBit
        self.RxAtrMask = 1 << self.RxAtrGpioBit
        self.AtrGpioMask = self.TxAtrMask | self.RxAtrMask

        self._configured_sample_rate = None
        self._initialised = False

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

        stream_args = uhd.usrp.StreamArgs(
            self.CpuFormat,
            self.WireFormat,
        )
        stream_args.channels = [self.Channel]
        self.RxStreamer = self.Usrp.get_rx_stream(stream_args)

        if self.AtrGpioEnabled:
            self._configure_atr_gpio()

        self._initialised = True
        print(
            "Ettus source initialised: "
            f"device='{args or 'auto'}', "
            f"RX={self.Usrp.get_rx_freq(self.Channel):.3f} Hz, "
            f"gain={self.Usrp.get_rx_gain(self.Channel):.2f} dB, "
            f"antenna={self.Usrp.get_rx_antenna(self.Channel)}, "
            f"ATR GPIO={'enabled' if self.AtrGpioEnabled else 'disabled'}"
        )

    def Shutdown(self):
        if self.RxStreamer is not None and uhd is not None:
            try:
                command = uhd.types.StreamCMD(
                    uhd.types.StreamMode.stop_cont
                )
                self.RxStreamer.issue_stream_cmd(command)
            except Exception:
                pass

        self.RxStreamer = None
        self.Usrp = None
        self._configured_sample_rate = None
        self._initialised = False
        print("Ettus source shutdown")

    def SetIqInjector(self, iq_injector: Optional[IqInjector]):
        """Set or clear the optional synthetic-target injection callback."""
        self.IqInjector = iq_injector

    def ExecuteDwell(self, ThisDwell):
        if not self._initialised:
            raise RuntimeError(
                "Call EttusRadarSource.Initialise() before ExecuteDwell()."
            )

        sample_rate = float(
            self.Config["EttusSampleRateHz"]
            )
        num_samples = int(ThisDwell.NumSamples)
        num_pulses = self._get_num_pulses(ThisDwell)

        if sample_rate <= 0.0:
            raise ValueError("SampleRate must be positive")
        if num_samples <= 0:
            raise ValueError("NumSamples must be positive")
        if num_pulses <= 0:
            raise ValueError("NumPulses must be positive")

        actual_sample_rate = self._configure_sample_rate(sample_rate)

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

        iq = np.zeros(
            (num_pulses, num_samples),
            dtype=np.complex64,
        )
        pulse_valid = np.ones(num_pulses, dtype=bool)
        pulse_diagnostics = []

        hardware_start_sec = (
            self.Usrp.get_time_now().get_real_secs()
            + self.CommandLeadTimeSec
        )
        wall_start = time.time()

        # Queue every timed RX command before waiting for any samples.
        # CommandLeadTimeSec is applied once to hardware_start_sec above;
        # subsequent PRI times are derived only from the dwell PRI schedule.
        scheduled_times_sec = hardware_start_sec + pulse_times_sec
        for scheduled_time_sec in scheduled_times_sec:
            self._issue_receive_command(
                num_samples=num_samples,
                scheduled_time_sec=float(scheduled_time_sec),
            )

        # After all receive windows are armed, collect one window per PRI.
        # This structure also leaves room to queue timed TX commands between
        # the RX-command loop above and the blocking receive loop below.
        for pulse_index in range(num_pulses):
            scheduled_time_sec = float(scheduled_times_sec[pulse_index])

            pulse_iq, diag = self._receive_scheduled_pri(
                num_samples=num_samples,
            )

            copied = min(len(pulse_iq), num_samples)
            if copied:
                iq[pulse_index, :copied] = pulse_iq[:copied]
            if copied != num_samples:
                pulse_valid[pulse_index] = False

            context = {
                "ScheduledHardwareTimeSec": float(scheduled_time_sec),
                "PulseTimeSec": float(pulse_times_sec[pulse_index]),
                "PriSec": float(pulse_pri_sec[pulse_index]),
                "WaveformId": str(pulse_waveform_ids[pulse_index]),
                "ValidBeforeInjection": bool(pulse_valid[pulse_index]),
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

        wall_end = time.time()

        diagnostics = {
            "SourceType": "EttusRadarSource",
            "ReceiveOnly": True,
            "ReceiveCommandPerPri": True,
            "AllReceiveCommandsQueuedBeforeCollection": True,
            "CommandLeadTimeAppliedOncePerDwell": True,
            "TimedTransmitEnabled": False,
            "SecondTransmitPulseEnabled": False,
            "SoftwareIqInjectionEnabled": self.IqInjector is not None,
            "RequestedSampleRate": sample_rate,
            "ActualSampleRate": actual_sample_rate,
            "NumSamplesPerPulse": num_samples,
            "NumPulses": num_pulses,
            "RxFrequencyHz": float(self.Usrp.get_rx_freq(self.Channel)),
            "RxGainDb": float(self.Usrp.get_rx_gain(self.Channel)),
            "RxAntenna": str(self.Usrp.get_rx_antenna(self.Channel)),
            "AtrGpioEnabled": bool(self.AtrGpioEnabled),
            "AtrGpioBank": str(self.GpioBank),
            "TxAtrGpioBit": int(self.TxAtrGpioBit),
            "RxAtrGpioBit": int(self.RxAtrGpioBit),
            "PulseDiagnostics": pulse_diagnostics,
            "CaptureElapsedSec": wall_end - wall_start,
        }

        return RawDwellData(
            DwellId=int(ThisDwell.DwellId),
            IQ=iq,
            SampleRate=float(actual_sample_rate),
            PRI=float(pulse_pri_sec[0]),
            TimeStamp=float(wall_end),
            PulseTimesSec=pulse_times_sec,
            PulsePriSec=pulse_pri_sec,
            PulseWaveformIds=pulse_waveform_ids,
            PulseValid=pulse_valid,
            Diagnostics=diagnostics,
        )

    def _configure_atr_gpio(self):
        """Configure J6 pins 3 and 4 as FPGA-controlled ATR outputs."""
        available_banks = list(self.Usrp.get_gpio_banks(0))
        if self.GpioBank not in available_banks:
            raise RuntimeError(
                f"GPIO bank '{self.GpioBank}' is unavailable; "
                f"available banks are {available_banks}"
            )

        if self.TxAtrGpioBit == self.RxAtrGpioBit:
            raise ValueError(
                "EttusTxAtrGPIO and EttusRxAtrGPIO must use different bits"
            )
        if self.TxAtrGpioBit < 0 or self.RxAtrGpioBit < 0:
            raise ValueError(
                "ATR GPIO bit numbers must be non-negative"
            )

        # CTRL=1 selects FPGA ATR control; DDR=1 selects output direction.
        self.Usrp.set_gpio_attr(
            self.GpioBank,
            "CTRL",
            self.AtrGpioMask,
            self.AtrGpioMask,
            0,
        )
        self.Usrp.set_gpio_attr(
            self.GpioBank,
            "DDR",
            self.AtrGpioMask,
            self.AtrGpioMask,
            0,
        )

        # ATR states for the Vanguard X half-duplex radar:
        # idle: both low; RX: pin 4 high; TX: pin 3 high; TX/RX: both low.
        self.Usrp.set_gpio_attr(
            self.GpioBank, "ATR_0X", 0, self.AtrGpioMask, 0
        )
        self.Usrp.set_gpio_attr(
            self.GpioBank,
            "ATR_RX",
            self.RxAtrMask,
            self.AtrGpioMask,
            0,
        )
        self.Usrp.set_gpio_attr(
            self.GpioBank,
            "ATR_TX",
            self.TxAtrMask,
            self.AtrGpioMask,
            0,
        )
        self.Usrp.set_gpio_attr(
            self.GpioBank, "ATR_XX", 0, self.AtrGpioMask, 0
        )

        print(
            "Ettus ATR GPIO configured: "
            f"bank={self.GpioBank}, "
            f"J6 pin 3=TX (GPIO_{self.TxAtrGpioBit}), "
            f"J6 pin 4=RX (GPIO_{self.RxAtrGpioBit})"
        )

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
        other_error_count = 0
        last_error = "none"
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

            last_error = self._error_name(metadata.error_code)

            if metadata.error_code == uhd.types.RXMetadataErrorCode.none:
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

            if metadata.error_code == uhd.types.RXMetadataErrorCode.timeout:
                timeout_count += 1
                break

            if metadata.error_code == uhd.types.RXMetadataErrorCode.overflow:
                overflow_count += 1
                continue

            late_code = getattr(
                uhd.types.RXMetadataErrorCode,
                "late_command",
                None,
            )
            if late_code is not None and metadata.error_code == late_code:
                late_command_count += 1
                break

            other_error_count += 1
            break

        diagnostics = {
            "RequestedSamples": int(num_samples),
            "ReceivedSamples": int(received_total),
            "ReceiveCalls": int(recv_calls),
            "TimeoutCount": int(timeout_count),
            "OverflowCount": int(overflow_count),
            "LateCommandCount": int(late_command_count),
            "OtherErrorCount": int(other_error_count),
            "LastMetadataError": str(last_error),
            "FirstSampleHardwareTimeSec": first_sample_time_sec,
        }

        return output[:received_total].copy(), diagnostics

    def _configure_sample_rate(self, requested_rate):
        if self._configured_sample_rate != requested_rate:
            self.Usrp.set_rx_rate(float(requested_rate), self.Channel)
            self._configured_sample_rate = float(
                self.Usrp.get_rx_rate(self.Channel)
            )
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
    def _error_name(error_code):
        return str(error_code).split(".")[-1].lower()
