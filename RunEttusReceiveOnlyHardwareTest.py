"""Safe receive-only Vanguard X hardware smoke test for an Ettus B200mini.

This program never creates a TX streamer, never sends waveform samples and
never calls the TRM controller. ATR GPIO control is disabled unless the
operator explicitly supplies ``--enable-atr-gpio``.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, replace
import sys
import traceback

import numpy as np

from EttusRadarSource import EttusRadarSource
from RadarPlans import make_uniform_dwell_plan
from RadarTiming import CalculateRadarTiming
from WaveformLibrary import WaveformLibrary


@dataclass
class HardwareEvaluation:
    Failures: list
    Warnings: list
    ReceivedSamples: int
    TimeoutCount: int
    OverflowCount: int
    LateCommandCount: int
    BrokenChainCount: int
    AlignmentErrorCount: int
    BadPacketCount: int
    OtherErrorCount: int
    MaximumTimestampErrorSec: float | None
    MinimumSignedTimestampErrorSec: float | None
    MeanSignedTimestampErrorSec: float | None
    MaximumSignedTimestampErrorSec: float | None
    IqRms: float
    IqPeak: float

    @property
    def Passed(self):
        return len(self.Failures) == 0


def BuildConfig(Arguments):
    Config = {
        "EttusOperatingMode": "RECEIVE_ONLY",
        "EttusTimedTransmitEnabled": False,
        "EttusSerial": str(Arguments.serial),
        "EttusRxFrequencyHz": float(Arguments.frequency_mhz) * 1.0e6,
        "EttusRxGainDb": float(Arguments.gain_db),
        "EttusRxAntenna": str(Arguments.antenna),
        "EttusRxChannel": int(Arguments.channel),
        "EttusSampleRateHz": 40.0e6,
        "EttusCpuFormat": "fc32",
        "EttusWireFormat": "sc16",
        "EttusReceiveTimeoutSec": float(Arguments.timeout_sec),
        "EttusCommandLeadTimeSec": float(Arguments.lead_ms) / 1000.0,
        "EttusCommandQueueDepth": 20,
        "EttusRxWarmupEnabled": True,
        "EttusAtrGpioEnabled": bool(Arguments.enable_atr_gpio),
        "EttusGPIOBank": "FP0",
        "EttusTxAtrGPIO": 1,
        "EttusRxAtrGPIO": 2,
        "EttusDebug": bool(Arguments.debug),
        "RfFrequency": 9.4e9,
    }
    if str(Arguments.device_args).strip():
        Config["EttusDeviceArgs"] = str(Arguments.device_args).strip()
    return Config


def BuildReceiveOnlyPlan(Timing, DwellId=1):
    """Build a dwell whose pulse metadata explicitly disables transmit."""

    Plan = make_uniform_dwell_plan(
        dwell_id=int(DwellId),
        task_id=0,
        task_type="HARDWARE_RX_TEST",
        waveform_id=Timing.WaveformId,
        sample_rate=Timing.SampleRateHz,
        num_samples=Timing.NumRxSamples,
        num_pulses=Timing.PulsesPerCpi,
        pri_sec=Timing.PriSec,
        rx_start_delay_sec=Timing.RxStartDelaySec,
        metadata={
            "HardwareReceiveOnlyTest": True,
            "TimedTransmitPermitted": False,
            "RadarTiming": Timing.ToMetadata(),
        },
    )
    Plan.PulsePlans = [
        replace(Pulse, TxEnabled=False)
        for Pulse in Plan.PulsePlans
    ]
    return Plan


def EvaluateHardwareDwell(Raw, Plan, Timing):
    Failures = []
    Warnings = []
    Diagnostics = dict(getattr(Raw, "Diagnostics", {}) or {})
    PulseDiagnostics = list(Diagnostics.get("PulseDiagnostics", []))

    ExpectedShape = (Timing.PulsesPerCpi, Timing.NumRxSamples)
    if tuple(Raw.IQ.shape) != ExpectedShape:
        Failures.append(
            f"IQ shape {Raw.IQ.shape} does not match {ExpectedShape}"
        )
    if not np.all(np.isfinite(Raw.IQ)):
        Failures.append("IQ contains non-finite values")

    if any(bool(Pulse.TxEnabled) for Pulse in Plan.PulsePlans):
        Failures.append("Receive-only plan contains a TX-enabled pulse")
    if Diagnostics.get("ReceiveOnly") is not True:
        Failures.append("Source did not declare ReceiveOnly=True")
    if Diagnostics.get("OperatingMode") != "RECEIVE_ONLY":
        Failures.append("Source did not declare OperatingMode=RECEIVE_ONLY")
    if Diagnostics.get("TimedTransmitEnabled") is not False:
        Failures.append("Source did not declare TimedTransmitEnabled=False")
    if Diagnostics.get("SoftwareIqInjectionEnabled") is not False:
        Failures.append("Software IQ injection was unexpectedly enabled")
    if Diagnostics.get("SBandStylePerPriLoop") is not True:
        Failures.append("Source did not use the S-band-style per-PRI loop")
    if Diagnostics.get("BoundedIndividualPriPipeline") is not True:
        Failures.append("Source did not use the bounded individual-PRI pipeline")
    if Diagnostics.get("ContinuousCpiCapture") is True:
        Failures.append("Source unexpectedly used continuous CPI capture")
    if int(Diagnostics.get("ReceiveCommandCount", 0)) != Timing.PulsesPerCpi:
        Failures.append(
            "Receive-only dwell did not issue one RX command per PRI"
        )
    ExpectedDepth = min(20, Timing.PulsesPerCpi)
    if (
        int(Diagnostics.get("ConfiguredCommandQueueDepth", 0)) != 20
    ):
        Failures.append("Bounded pipeline was not configured for depth 20")
    if (
        int(Diagnostics.get("ActiveCommandQueueDepth", 0)) != ExpectedDepth
    ):
        Failures.append("Bounded pipeline calculated an incorrect active depth")
    if (
        int(Diagnostics.get("MaximumOutstandingReceiveCommands", 0))
        != ExpectedDepth
    ):
        Failures.append("Bounded pipeline exceeded or failed to fill depth 20")
    if int(Diagnostics.get("HardwareTimeQueriesPerDwell", 0)) != 1:
        Failures.append("Source queried hardware time inside the PRI loop")
    if any(
        bool(Item.get("TransmitQueued", False))
        for Item in PulseDiagnostics
    ):
        Failures.append("Receive-only per-PRI loop unexpectedly queued TX")

    if not np.isclose(
        float(Raw.SampleRate),
        float(Timing.SampleRateHz),
        rtol=0.0,
        atol=1.0,
    ):
        Failures.append(
            f"Actual sample rate {Raw.SampleRate:g} Hz does not match "
            f"{Timing.SampleRateHz:g} Hz"
        )

    PulseValid = np.asarray(Raw.PulseValid, dtype=bool)
    if PulseValid.shape != (Timing.PulsesPerCpi,):
        Failures.append(
            f"PulseValid shape {PulseValid.shape} is incorrect"
        )
    elif not np.all(PulseValid):
        Invalid = np.flatnonzero(~PulseValid).tolist()
        Failures.append(f"Invalid receive pulses: {Invalid}")

    if len(PulseDiagnostics) != Timing.PulsesPerCpi:
        Failures.append(
            f"Received {len(PulseDiagnostics)} pulse diagnostics; "
            f"expected {Timing.PulsesPerCpi}"
        )

    ReceivedSamples = sum(
        int(Item.get("ReceivedSamples", 0))
        for Item in PulseDiagnostics
    )
    TimeoutCount = sum(
        int(Item.get("TimeoutCount", 0))
        for Item in PulseDiagnostics
    )
    OverflowCount = sum(
        int(Item.get("OverflowCount", 0))
        for Item in PulseDiagnostics
    )
    LateCommandCount = sum(
        int(Item.get("LateCommandCount", 0))
        for Item in PulseDiagnostics
    )
    BrokenChainCount = sum(
        int(Item.get("BrokenChainCount", 0))
        for Item in PulseDiagnostics
    )
    AlignmentErrorCount = sum(
        int(Item.get("AlignmentErrorCount", 0))
        for Item in PulseDiagnostics
    )
    BadPacketCount = sum(
        int(Item.get("BadPacketCount", 0))
        for Item in PulseDiagnostics
    )
    OtherErrorCount = sum(
        int(Item.get("OtherErrorCount", 0))
        for Item in PulseDiagnostics
    )

    ExpectedReceivedSamples = Timing.PulsesPerCpi * Timing.NumRxSamples
    if ReceivedSamples != ExpectedReceivedSamples:
        Failures.append(
            f"Received {ReceivedSamples} total samples; "
            f"expected {ExpectedReceivedSamples}"
        )
    if TimeoutCount:
        Failures.append(f"UHD reported {TimeoutCount} receive timeouts")
    if OverflowCount:
        Failures.append(f"UHD reported {OverflowCount} overflows")
    if LateCommandCount:
        Failures.append(f"UHD reported {LateCommandCount} late commands")
    if BrokenChainCount:
        Failures.append(f"UHD reported {BrokenChainCount} broken chains")
    if AlignmentErrorCount:
        Failures.append(f"UHD reported {AlignmentErrorCount} alignment errors")
    if BadPacketCount:
        Failures.append(f"UHD reported {BadPacketCount} bad packets")
    if OtherErrorCount:
        Failures.append(f"UHD reported {OtherErrorCount} other errors")

    ScheduledRxTimes = np.asarray(
        Diagnostics.get("ScheduledRxHardwareTimesSec", []),
        dtype=np.float64,
    )
    if ScheduledRxTimes.shape != (Timing.PulsesPerCpi,):
        Failures.append("Scheduled RX hardware-time array is incomplete")
    elif Timing.PulsesPerCpi > 1 and not np.allclose(
        np.diff(ScheduledRxTimes),
        Timing.PriSec,
        rtol=0.0,
        atol=1.0e-9,
    ):
        Failures.append("Scheduled RX windows do not follow the selected PRI")

    TimestampErrors = []
    for Item in PulseDiagnostics:
        ActualTime = Item.get("FirstSampleHardwareTimeSec")
        ScheduledTime = Item.get("ScheduledHardwareTimeSec")
        if ActualTime is not None and ScheduledTime is not None:
            TimestampErrors.append(float(ActualTime) - float(ScheduledTime))

    MaximumTimestampErrorSec = None
    MinimumSignedTimestampErrorSec = None
    MeanSignedTimestampErrorSec = None
    MaximumSignedTimestampErrorSec = None
    if TimestampErrors:
        TimestampErrors = np.asarray(TimestampErrors, dtype=np.float64)
        MaximumTimestampErrorSec = float(np.max(np.abs(TimestampErrors)))
        MinimumSignedTimestampErrorSec = float(np.min(TimestampErrors))
        MeanSignedTimestampErrorSec = float(np.mean(TimestampErrors))
        MaximumSignedTimestampErrorSec = float(np.max(TimestampErrors))
        if MaximumTimestampErrorSec > 2.0e-6:
            Failures.append(
                "Maximum first-sample timestamp error is "
                f"{MaximumTimestampErrorSec * 1e6:.3f} us"
            )
        elif MaximumTimestampErrorSec > 1.0 / Timing.SampleRateHz:
            Warnings.append(
                "First-sample timestamp error exceeds one sample: "
                f"{MaximumTimestampErrorSec * 1e9:.1f} ns"
            )
    else:
        Warnings.append("UHD did not provide first-sample timestamps")

    IqMagnitude = np.abs(np.asarray(Raw.IQ, dtype=np.complex64))
    if IqMagnitude.size:
        IqRms = float(
            np.sqrt(np.mean(IqMagnitude.astype(np.float64) ** 2))
        )
        IqPeak = float(np.max(IqMagnitude))
    else:
        IqRms = 0.0
        IqPeak = 0.0

    return HardwareEvaluation(
        Failures=Failures,
        Warnings=Warnings,
        ReceivedSamples=ReceivedSamples,
        TimeoutCount=TimeoutCount,
        OverflowCount=OverflowCount,
        LateCommandCount=LateCommandCount,
        BrokenChainCount=BrokenChainCount,
        AlignmentErrorCount=AlignmentErrorCount,
        BadPacketCount=BadPacketCount,
        OtherErrorCount=OtherErrorCount,
        MaximumTimestampErrorSec=MaximumTimestampErrorSec,
        MinimumSignedTimestampErrorSec=MinimumSignedTimestampErrorSec,
        MeanSignedTimestampErrorSec=MeanSignedTimestampErrorSec,
        MaximumSignedTimestampErrorSec=MaximumSignedTimestampErrorSec,
        IqRms=IqRms,
        IqPeak=IqPeak,
    )


def PrintTiming(Timing, Arguments):
    print("Receive-only timing plan")
    print(f"  waveform:       {Timing.WaveformId}")
    print(f"  sample rate:    {Timing.SampleRateHz / 1e6:.3f} MS/s")
    print(f"  PRF / PRI:      {Timing.SelectedPrfHz:.0f} Hz / {Timing.PriSec * 1e6:.3f} us")
    print(f"  pulses / CPI:   {Timing.PulsesPerCpi} / {Timing.CpiDurationSec * 1e3:.3f} ms")
    print(f"  maximum range:  {Timing.MaximumRangeM / 1e3:.3f} km")
    print(f"  RX start:       {Timing.RxStartDelaySec * 1e6:.3f} us")
    print(f"  RX samples:     {Timing.NumRxSamples} per pulse")
    ActiveDepth = min(20, Timing.PulsesPerCpi)
    print(
        "  RX execution:   bounded individual PRI pipeline, "
        f"depth={ActiveDepth}"
    )
    print(
        "  queue horizon:  "
        f"{ActiveDepth * Timing.PriSec * 1e3:.3f} ms"
    )
    print("  RX warm-up:     enabled once before first dwell T0")
    print(f"  command lead:   {Arguments.lead_ms:.3f} ms")
    print(f"  ATR GPIO:       {'ENABLED' if Arguments.enable_atr_gpio else 'DISABLED'}")
    print("  timed transmit: DISABLED")


def PrintEvaluation(DwellIndex, Raw, Evaluation):
    Status = "PASS" if Evaluation.Passed else "FAIL"
    print(f"Dwell {DwellIndex}: {Status}")
    print(f"  IQ shape:       {Raw.IQ.shape}")
    print(f"  sample rate:    {Raw.SampleRate / 1e6:.6f} MS/s")
    print(f"  samples total:  {Evaluation.ReceivedSamples}")
    if Raw.Diagnostics.get("BoundedIndividualPriPipeline"):
        CommandCount = int(Raw.Diagnostics.get("ReceiveCommandCount", 0))
        MaximumOutstanding = int(
            Raw.Diagnostics.get("MaximumOutstandingReceiveCommands", 0)
        )
        ActiveDepth = int(
            Raw.Diagnostics.get("ActiveCommandQueueDepth", 0)
        )
        print(
            f"  UHD RX commands:{CommandCount:4d} individual finite windows; "
            f"depth={ActiveDepth}, "
            f"max outstanding={MaximumOutstanding}"
        )
    if Raw.Diagnostics.get("RxWarmupPerformed"):
        Warmup = dict(Raw.Diagnostics.get("RxWarmupDiagnostics") or {})
        print(
            "  RX warm-up:     PASS, "
            f"samples={Warmup.get('ReceivedSamples', 0)}/"
            f"{Warmup.get('RequestedSamples', 0)}"
        )
    print(
        "  UHD errors:     "
        f"timeout={Evaluation.TimeoutCount}, "
        f"overflow={Evaluation.OverflowCount}, "
        f"late={Evaluation.LateCommandCount}, "
        f"broken={Evaluation.BrokenChainCount}, "
        f"align={Evaluation.AlignmentErrorCount}, "
        f"badpkt={Evaluation.BadPacketCount}, "
        f"other={Evaluation.OtherErrorCount}"
    )
    if Evaluation.MaximumTimestampErrorSec is None:
        print("  timestamp err:  unavailable")
    else:
        print(
            "  timestamp err:  "
            f"{Evaluation.MinimumSignedTimestampErrorSec * 1e9:+.1f} / "
            f"{Evaluation.MeanSignedTimestampErrorSec * 1e9:+.1f} / "
            f"{Evaluation.MaximumSignedTimestampErrorSec * 1e9:+.1f} ns "
            "min/mean/max"
        )
    print(f"  IQ RMS / peak:  {Evaluation.IqRms:.6g} / {Evaluation.IqPeak:.6g}")
    print(
        "  capture wall:   "
        f"{float(Raw.Diagnostics.get('CaptureElapsedSec', 0.0)) * 1e3:.3f} ms"
    )
    for PulseIndex, Item in enumerate(
        Raw.Diagnostics.get("PulseDiagnostics", [])
    ):
        Requested = int(Item.get("RequestedSamples", 0))
        Received = int(Item.get("ReceivedSamples", 0))
        ErrorEvents = list(Item.get("MetadataErrorEvents", []))
        if Received != Requested or ErrorEvents:
            ErrorName = str(Item.get("LastMetadataError", "none"))
            ErrorCode = Item.get("LastMetadataErrorCodeValue", 0)
            ErrorText = str(Item.get("LastMetadataErrorText", "")).strip()
            Detail = f", text={ErrorText}" if ErrorText else ""
            print(
                f"  pulse error:    index={PulseIndex}, "
                f"samples={Received}/{Requested}, "
                f"metadata={ErrorName}({ErrorCode}){Detail}"
            )
    for Warning in Evaluation.Warnings:
        print(f"  WARNING: {Warning}")
    for Failure in Evaluation.Failures:
        print(f"  FAILURE: {Failure}")


def ParseArguments(Argv=None):
    Parser = argparse.ArgumentParser(
        description="Run a transmitter-inhibited Ettus receive-only dwell",
    )
    Parser.add_argument("--serial", default="34A0320")
    Parser.add_argument("--device-args", default="")
    Parser.add_argument("--frequency-mhz", type=float, default=1000.0)
    Parser.add_argument("--gain-db", type=float, default=10.0)
    Parser.add_argument("--antenna", default="RX2")
    Parser.add_argument("--channel", type=int, default=0)
    Parser.add_argument("--waveform", default="Frank10_20MHz")
    Parser.add_argument("--prf-hz", type=float, default=2000.0)
    Parser.add_argument("--pulses", type=int, default=32)
    Parser.add_argument("--max-range-km", type=float, default=15.0)
    Parser.add_argument("--lead-ms", type=float, default=50.0)
    Parser.add_argument("--timeout-sec", type=float, default=1.0)
    Parser.add_argument("--dwells", type=int, default=1)
    Parser.add_argument("--enable-atr-gpio", action="store_true")
    Parser.add_argument("--debug", action="store_true")
    return Parser.parse_args(Argv)


def ValidateArguments(Arguments):
    if Arguments.lead_ms < 20.0:
        raise ValueError(
            "Initial hardware validation requires at least 20 ms command lead"
        )
    if Arguments.timeout_sec <= 0.0:
        raise ValueError("Receive timeout must be positive")
    if Arguments.dwells <= 0:
        raise ValueError("Dwell count must be positive")
    if Arguments.max_range_km <= 0.0:
        raise ValueError("Maximum range must be positive")


def Main(Argv=None):
    Arguments = ParseArguments(Argv)
    ValidateArguments(Arguments)
    Config = BuildConfig(Arguments)

    Library = WaveformLibrary(Config)
    Library.LoadDefaultWaveforms()
    Timing = CalculateRadarTiming(
        Library.GetMetadata(Arguments.waveform),
        SelectedPrfHz=Arguments.prf_hz,
        PulsesPerCpi=Arguments.pulses,
        MaximumRangeM=Arguments.max_range_km * 1000.0,
        ReceiverRecoveryTimeSec=1.0e-6,
        RxEndMarginSec=2.0e-6,
        NextTxGuardTimeSec=2.0e-6,
        OperatorMinPrfHz=1000.0,
        OperatorMaxPrfHz=4000.0,
        RfFrequencyHz=9.4e9,
        AntennaScanRateDegPerSec=0.0,
    )
    PrintTiming(Timing, Arguments)

    Source = EttusRadarSource(Config, Library)
    OverallPassed = True
    try:
        Source.Initialise()
        for DwellIndex in range(1, Arguments.dwells + 1):
            Plan = BuildReceiveOnlyPlan(Timing, DwellId=DwellIndex)
            Raw = Source.ExecuteDwell(Plan)
            Evaluation = EvaluateHardwareDwell(Raw, Plan, Timing)
            PrintEvaluation(DwellIndex, Raw, Evaluation)
            OverallPassed = OverallPassed and Evaluation.Passed
            if not Evaluation.Passed:
                break
    except Exception:
        OverallPassed = False
        print("Receive-only hardware test raised an exception:")
        traceback.print_exc()
    finally:
        Source.Shutdown()

    print(
        "Receive-only hardware test "
        f"{'PASSED' if OverallPassed else 'FAILED'}"
    )
    return 0 if OverallPassed else 1


if __name__ == "__main__":
    sys.exit(Main())
