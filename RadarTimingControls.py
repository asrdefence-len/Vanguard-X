"""Dwell-boundary application of operator timing selections."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class TimingControlApplication:
    """Result of consuming one UI timing-selection revision."""

    Revision: int
    Changed: bool
    Applied: bool
    Message: str
    Profile: object = None


def ApplyTimingControlState(
    Config,
    ControlState,
    Executor,
    SearchTask,
    LastAppliedRevision,
):
    """Validate and apply a UI selection for the next search dwell.

    The Config dictionary is updated only when RadarExecutor can construct a
    complete validated timing profile. A failed selection is rolled back.
    """

    if ControlState is None or "TimingSelectionRevision" not in ControlState:
        return TimingControlApplication(
            Revision=int(LastAppliedRevision),
            Changed=False,
            Applied=False,
            Message="No timing controls",
        )

    try:
        Revision = int(ControlState["TimingSelectionRevision"])
    except (TypeError, ValueError) as Error:
        return TimingControlApplication(
            Revision=int(LastAppliedRevision),
            Changed=True,
            Applied=False,
            Message=f"Invalid timing selection revision: {Error}",
        )
    if Revision == int(LastAppliedRevision):
        return TimingControlApplication(
            Revision=Revision,
            Changed=False,
            Applied=False,
            Message="Timing selection unchanged",
        )

    try:
        WaveformId = str(
            ControlState.get("SelectedWaveformId", "")
        ).strip()
        PrfHz = float(ControlState.get("SelectedPrfHz", 0.0))
        RawPulses = ControlState.get("SelectedPulsesPerCpi", 0)
        PulsesPerCpi = int(RawPulses)
        MaximumRangeM = float(ControlState.get(
            "SelectedMaximumRangeM",
            Config.get("InstrumentedMaxRangeM", 15000.0),
        ))

        if not WaveformId:
            raise ValueError("Selected waveform must not be empty")
        if not math.isfinite(PrfHz) or PrfHz <= 0.0:
            raise ValueError("Selected PRF must be positive and finite")
        if (
            isinstance(RawPulses, bool)
            or PulsesPerCpi <= 0
            or float(RawPulses) != float(PulsesPerCpi)
        ):
            raise ValueError(
                "Selected pulses per CPI must be a positive integer"
            )
        if not math.isfinite(MaximumRangeM) or MaximumRangeM <= 0.0:
            raise ValueError(
                "Selected maximum range must be positive and finite"
            )
    except (TypeError, ValueError) as Error:
        return TimingControlApplication(
            Revision=Revision,
            Changed=True,
            Applied=False,
            Message=str(Error),
        )

    Keys = (
        "SearchWaveformId",
        "SearchPrfHz",
        "SearchPulsesPerCpi",
        "SearchMaximumRangeM",
        "SelectedPrfHz",
        "SelectedPulsesPerCpi",
        "InstrumentedMaxRangeM",
    )
    Missing = object()
    Previous = {Key: Config.get(Key, Missing) for Key in Keys}

    Config["SearchWaveformId"] = WaveformId
    Config["SearchPrfHz"] = PrfHz
    Config["SearchPulsesPerCpi"] = PulsesPerCpi
    Config["SearchMaximumRangeM"] = MaximumRangeM
    Config["SelectedPrfHz"] = PrfHz
    Config["SelectedPulsesPerCpi"] = PulsesPerCpi
    Config["InstrumentedMaxRangeM"] = MaximumRangeM

    try:
        Profile = Executor.GetExecutionProfile(SearchTask)
    except Exception as Error:
        for Key, Value in Previous.items():
            if Value is Missing:
                Config.pop(Key, None)
            else:
                Config[Key] = Value
        return TimingControlApplication(
            Revision=Revision,
            Changed=True,
            Applied=False,
            Message=str(Error),
        )

    return TimingControlApplication(
        Revision=Revision,
        Changed=True,
        Applied=True,
        Message="Applied for next dwell",
        Profile=Profile,
    )
