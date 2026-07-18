"""
===============================================================================
ASR Defence X-Band Radar Prototype
SimulatedSource.py
===============================================================================

Foreword
--------
This file implements the simulated radar source.

The simulated source allows the radar software to be developed without the Ettus
radio connected.

Current simulation behaviour
----------------------------
Current version:
    - creates complex receiver noise
    - supports either the original single target mode or a scene-return list
    - calculates received target power using the monostatic radar equation
    - applies the two-way antenna pattern gain for scene objects
    - adds Doppler phase progression across pulses
    - returns raw IQ as a 2D array:

          NumPulses x NumSamples

Radar equation note
-------------------
The received power for a monostatic radar is:

    Pr = Pt * Gt * Gr * Lambda^2 * RCS / ((4*pi)^3 * R^4 * L)

For the same antenna on transmit and receive:

    Gt = Gr = G

The simulated target amplitude is proportional to sqrt(Pr), because IQ samples
represent complex voltage-like amplitude, while radar equation output is power.

Scene-return mode
-----------------
If Config["SceneReturns"] exists and is a non-empty list, every return in that
list is inserted into the simulated receive buffer. This is the new scanning
scene model. Each return should contain at least:

    range_m
    doppler_hz
    rcs
    two_way_gain_power

This allows all reflectors to contribute through the antenna main beam and
sidelobes.
===============================================================================
"""

import time
import numpy as np
from DataTypes import RawDwellData


class SimulatedSource:
    """
    Simulated radar source.

    This creates synthetic raw IQ data for each dwell.
    """

    def __init__(self, Config, TheWaveformLibrary):
        self.Config = Config
        self.TheWaveformLibrary = TheWaveformLibrary

    def Initialise(self):
        """
        Initialise the simulated source.
        """

        print("Simulated source initialised")

    def DbToLinear(self, ValueDb):
        """
        Convert dB to linear power ratio.
        """

        return 10.0 ** (ValueDb / 10.0)

    def CalculateReceivedPowerW(
        self,
        TransmitPowerW,
        AntennaGainDb,
        FrequencyHz,
        TargetRcsSqm,
        TargetRangeM,
        SystemLossDb,
        TwoWayPatternGainPower=1.0,
    ):
        """
        Calculate received power using the monostatic radar equation.

        TwoWayPatternGainPower is a normalised two-way antenna pattern multiplier.
        It should be 1.0 on boresight and smaller in sidelobes.
        """

        SpeedOfLight = 299792458.0

        TargetRangeM = max(float(TargetRangeM), 1.0)

        WavelengthM = SpeedOfLight / FrequencyHz
        AntennaGainLinear = self.DbToLinear(AntennaGainDb)
        SystemLossLinear = self.DbToLinear(SystemLossDb)

        Numerator = (
            TransmitPowerW
            * AntennaGainLinear
            * AntennaGainLinear
            * WavelengthM ** 2
            * TargetRcsSqm
            * TwoWayPatternGainPower
        )

        Denominator = (
            (4.0 * np.pi) ** 3
            * TargetRangeM ** 4
            * SystemLossLinear
        )

        ReceivedPowerW = Numerator / Denominator

        return ReceivedPowerW

    def BuildDefaultSingleTargetSceneReturns(self, RfFrequency):
        """
        Backwards-compatible helper.

        If the main program has not provided Config["SceneReturns"], create one
        return from the original single-target config fields.
        """

        SpeedOfLight = 299792458.0
        WavelengthM = SpeedOfLight / RfFrequency

        TargetRangeM = self.Config["TargetRangeM"]
        TargetVelocityMps = self.Config["TargetVelocityMps"]
        TargetRcsSqm = self.Config["TargetRcsSqm"]
        TargetDopplerHz = 2.0 * TargetVelocityMps / WavelengthM

        return [
            {
                "name": "Single_Target_Backwards_Compatible",
                "range_m": TargetRangeM,
                "bearing_deg": 0.0,
                "angle_error_deg": 0.0,
                "radial_velocity_mps": TargetVelocityMps,
                "doppler_hz": TargetDopplerHz,
                "rcs": TargetRcsSqm,
                "one_way_gain_power": 1.0,
                "two_way_gain_power": 1.0,
                "two_way_gain_db": 0.0,
            }
        ]


    @staticmethod
    def _GetNumPulses(ThisDwell):
        if hasattr(ThisDwell, "PulsePlans"):
            return len(ThisDwell.PulsePlans)
        return int(ThisDwell.NumPulses)

    @staticmethod
    def _GetPulseWaveformId(ThisDwell, PulseIndex):
        if hasattr(ThisDwell, "PulsePlans"):
            return str(ThisDwell.PulsePlans[PulseIndex].WaveformId)
        return str(ThisDwell.WaveformName)

    @staticmethod
    def _GetPulsePriSec(ThisDwell, PulseIndex):
        if hasattr(ThisDwell, "PulsePlans"):
            return float(ThisDwell.PulsePlans[PulseIndex].PriSec)
        return float(ThisDwell.PRI)

    @staticmethod
    def _GetPulseRxStartDelaySec(ThisDwell, PulseIndex):
        if hasattr(ThisDwell, "PulsePlans"):
            return float(ThisDwell.PulsePlans[PulseIndex].RxStartDelaySec)
        return 0.0

    @staticmethod
    def _GetPulseEnabled(ThisDwell, PulseIndex):
        if hasattr(ThisDwell, "PulsePlans"):
            return bool(ThisDwell.PulsePlans[PulseIndex].TxEnabled)
        return True

    @staticmethod
    def _GetPulseAmplitudeScale(ThisDwell, PulseIndex):
        if hasattr(ThisDwell, "PulsePlans"):
            return float(ThisDwell.PulsePlans[PulseIndex].AmplitudeScale)
        return 1.0

    @staticmethod
    def _GetPulsePhaseOffsetRad(ThisDwell, PulseIndex):
        if hasattr(ThisDwell, "PulsePlans"):
            return float(ThisDwell.PulsePlans[PulseIndex].PhaseOffsetRad)
        return 0.0

    def ExecuteDwell(self, ThisDwell):
        """
        Execute one simulated dwell.

        The output IQ array has shape:

            NumPulses x NumSamples
        """

        SampleRate = ThisDwell.SampleRate
        NumSamples = ThisDwell.NumSamples
        NumPulses = self._GetNumPulses(ThisDwell)

        PulsePriSec = np.asarray(
            [self._GetPulsePriSec(ThisDwell, i) for i in range(NumPulses)],
            dtype=np.float64,
        )
        PulseTimesSec = np.zeros(NumPulses, dtype=np.float64)
        if NumPulses > 1:
            PulseTimesSec[1:] = np.cumsum(PulsePriSec[:-1])

        PulseRxStartDelaySec = np.asarray(
            [
                self._GetPulseRxStartDelaySec(ThisDwell, i)
                for i in range(NumPulses)
            ],
            dtype=np.float64,
        )
        if np.any(PulseRxStartDelaySec < 0.0):
            raise ValueError("RX start delay must not be negative")

        PulseWaveformIds = [
            self._GetPulseWaveformId(ThisDwell, i)
            for i in range(NumPulses)
        ]
        PulseValid = np.ones(NumPulses, dtype=bool)
        PRI = float(PulsePriSec[0])

        TransmitPowerW = self.Config["TransmitPowerW"]
        AntennaGainDb = self.Config["AntennaGainDb"]
        SystemLossDb = self.Config["SystemLossDb"]
        RfFrequency = self.Config["RfFrequency"]
        NoisePowerW = self.Config["NoisePowerW"]

        SpeedOfLight = 299792458.0
        WavelengthM = SpeedOfLight / RfFrequency

        # ---------------------------------------------------------------------
        # Create 2D complex Gaussian receiver noise
        # ---------------------------------------------------------------------
        # NoisePowerW is the complex sample noise power.
        # Each of I and Q gets half the power.
        # ---------------------------------------------------------------------

        Noise = np.sqrt(NoisePowerW / 2.0) * (
            np.random.randn(NumPulses, NumSamples)
            + 1j * np.random.randn(NumPulses, NumSamples)
        )

        IQ = Noise.astype(np.complex64)

        # ---------------------------------------------------------------------
        # Get scene returns
        # ---------------------------------------------------------------------
        # New mode:
        #     Config["SceneReturns"] is provided by TargetScenario.py.
        # Old mode:
        #     Use the original single target config fields.
        # ---------------------------------------------------------------------

        SceneReturns = self.Config.get("SceneReturns", None)

        if SceneReturns is None or len(SceneReturns) == 0:
            SceneReturns = self.BuildDefaultSingleTargetSceneReturns(RfFrequency)

        ReturnDiagnostics = []
        TotalReceivedPowerW = 0.0

        # ---------------------------------------------------------------------
        # Insert every scene return into each pulse
        # ---------------------------------------------------------------------

        for ThisReturn in SceneReturns:
            TargetName = ThisReturn.get("name", "Unnamed_Return")
            TargetRangeM = float(ThisReturn["range_m"])
            TargetDopplerHz = float(ThisReturn["doppler_hz"])
            TargetRcsSqm = float(ThisReturn.get("rcs", 1.0))
            TwoWayPatternGainPower = float(ThisReturn.get("two_way_gain_power", 1.0))

            TargetDelayS = 2.0 * TargetRangeM / SpeedOfLight
            TargetDelaySamples = int(round(TargetDelayS * SampleRate))

            ReceivedPowerW = self.CalculateReceivedPowerW(
                TransmitPowerW=TransmitPowerW,
                AntennaGainDb=AntennaGainDb,
                FrequencyHz=RfFrequency,
                TargetRcsSqm=TargetRcsSqm,
                TargetRangeM=TargetRangeM,
                SystemLossDb=SystemLossDb,
                TwoWayPatternGainPower=TwoWayPatternGainPower,
            )

            TargetAmplitude = np.sqrt(ReceivedPowerW)
            TotalReceivedPowerW += ReceivedPowerW

            Inserted = False
            CaptureRelativeDelaySamples = []

            for PulseIndex in range(NumPulses):
                if not self._GetPulseEnabled(ThisDwell, PulseIndex):
                    CaptureRelativeDelaySamples.append(None)
                    continue

                WaveformId = PulseWaveformIds[PulseIndex]
                TxWaveform = self.TheWaveformLibrary.Get(WaveformId)
                RelativeDelaySec = (
                    TargetDelayS - PulseRxStartDelaySec[PulseIndex]
                )
                StartIndex = int(round(RelativeDelaySec * SampleRate))
                EndIndex = StartIndex + len(TxWaveform)
                CaptureRelativeDelaySamples.append(StartIndex)

                # A target outside the configured receive window does not make
                # the acquired pulse invalid. It simply contributes no return.
                if StartIndex < 0 or EndIndex > NumSamples:
                    continue

                Inserted = True
                PulseTime = PulseTimesSec[PulseIndex]
                DopplerPhase = (
                    2.0 * np.pi * TargetDopplerHz * PulseTime
                    + self._GetPulsePhaseOffsetRad(ThisDwell, PulseIndex)
                )
                DopplerMultiplier = np.exp(1j * DopplerPhase)
                AmplitudeScale = self._GetPulseAmplitudeScale(
                    ThisDwell, PulseIndex
                )

                IQ[PulseIndex, StartIndex:EndIndex] += (
                    TargetAmplitude
                    * AmplitudeScale
                    * DopplerMultiplier
                    * TxWaveform
                )

            if not Inserted:
                print(
                    f"Warning: simulated return {TargetName} at "
                    f"{TargetRangeM:.1f} m is outside receive window"
                )

            ReturnDiagnostics.append(
                {
                    "name": TargetName,
                    "range_m": TargetRangeM,
                    "bearing_deg": ThisReturn.get("bearing_deg", 0.0),
                    "angle_error_deg": ThisReturn.get("angle_error_deg", 0.0),
                    "radial_velocity_mps": ThisReturn.get("radial_velocity_mps", 0.0),
                    "doppler_hz": TargetDopplerHz,
                    "rcs": TargetRcsSqm,
                    "one_way_gain_power": ThisReturn.get("one_way_gain_power", 1.0),
                    "two_way_gain_power": TwoWayPatternGainPower,
                    "two_way_gain_db": ThisReturn.get(
                        "two_way_gain_db",
                        10.0 * np.log10(TwoWayPatternGainPower + 1e-300),
                    ),
                    "received_power_w": ReceivedPowerW,
                    "received_power_dbm": 10.0 * np.log10(ReceivedPowerW / 1e-3 + 1e-30),
                    "target_amplitude": TargetAmplitude,
                    "delay_samples": TargetDelaySamples,
                    "capture_relative_delay_samples": (
                        CaptureRelativeDelaySamples
                    ),
                    "inserted": Inserted,
                }
            )

        # ---------------------------------------------------------------------
        # Store diagnostics
        # ---------------------------------------------------------------------

        Diagnostics = {
            "TransmitPowerW": TransmitPowerW,
            "AntennaGainDb": AntennaGainDb,
            "SystemLossDb": SystemLossDb,
            "RfFrequency": RfFrequency,
            "WavelengthM": WavelengthM,
            "NoisePowerW": NoisePowerW,
            "NoisePowerDbm": 10.0 * np.log10(NoisePowerW / 1e-3 + 1e-30),
            "SceneReturns": ReturnDiagnostics,
            "NumSceneReturns": len(ReturnDiagnostics),
            "TotalReceivedPowerW": TotalReceivedPowerW,
            "TotalReceivedPowerDbm": 10.0 * np.log10(TotalReceivedPowerW / 1e-3 + 1e-30),
            "InputSnrDb": 10.0 * np.log10((TotalReceivedPowerW + 1e-30) / (NoisePowerW + 1e-30)),
            "BoresightDeg": self.Config.get("BoresightDeg", None),
            "BeamwidthDeg": self.Config.get("BeamwidthDeg", None),
        }

        # Keep the old diagnostic names populated for older display code.
        if len(ReturnDiagnostics) > 0:
            FirstReturn = ReturnDiagnostics[0]
            Diagnostics["TargetRangeM"] = FirstReturn["range_m"]
            Diagnostics["TargetVelocityMps"] = FirstReturn["radial_velocity_mps"]
            Diagnostics["TargetRcsSqm"] = FirstReturn["rcs"]
            Diagnostics["TargetDopplerHz"] = FirstReturn["doppler_hz"]
            Diagnostics["ReceivedPowerW"] = FirstReturn["received_power_w"]
            Diagnostics["ReceivedPowerDbm"] = FirstReturn["received_power_dbm"]

        Diagnostics["PulseWaveformIds"] = list(PulseWaveformIds)
        Diagnostics["PulsePriSec"] = PulsePriSec.copy()
        Diagnostics["PulseTimesSec"] = PulseTimesSec.copy()
        Diagnostics["PulseRxStartDelaySec"] = (
            PulseRxStartDelaySec.copy()
        )
        Diagnostics["FirstRxSampleRangeOffsetM"] = float(
            SpeedOfLight * PulseRxStartDelaySec[0] / 2.0
        )
        Diagnostics["PulseValid"] = PulseValid.copy()

        Raw = RawDwellData(
            DwellId=ThisDwell.DwellId,
            IQ=IQ,
            SampleRate=SampleRate,
            PRI=PRI,
            TimeStamp=time.time(),
            PulseTimesSec=PulseTimesSec,
            PulsePriSec=PulsePriSec,
            PulseRxStartDelaySec=PulseRxStartDelaySec,
            PulseWaveformIds=PulseWaveformIds,
            PulseValid=PulseValid,
            Diagnostics=Diagnostics,
        )

        return Raw

    def Shutdown(self):
        """
        Shut down the simulated source.
        """

        print("Simulated source shutdown")
