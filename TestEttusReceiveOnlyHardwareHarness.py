"""Non-hardware regression tests for the receive-only smoke-test harness."""

from types import SimpleNamespace
import unittest

import numpy as np

from RunEttusReceiveOnlyHardwareTest import (
    BuildConfig,
    BuildReceiveOnlyPlan,
    EvaluateHardwareDwell,
    ParseArguments,
    ValidateArguments,
)
from RadarTiming import CalculateRadarTiming
from WaveformLibrary import WaveformLibrary


class TestEttusReceiveOnlyHardwareHarness(unittest.TestCase):
    def setUp(self):
        self.Arguments = ParseArguments([])
        self.Config = BuildConfig(self.Arguments)
        self.Library = WaveformLibrary(self.Config)
        self.Library.LoadDefaultWaveforms()
        self.Timing = CalculateRadarTiming(
            self.Library.GetMetadata("Frank10_20MHz"),
            PulsesPerCpi=4,
        )
        self.Plan = BuildReceiveOnlyPlan(self.Timing)

    def MakeRaw(self):
        Scheduled = (
            100.0
            + self.Timing.RxStartDelaySec
            + np.arange(self.Timing.PulsesPerCpi) * self.Timing.PriSec
        )
        PulseDiagnostics = [
            {
                "RequestedSamples": self.Timing.NumRxSamples,
                "ReceivedSamples": self.Timing.NumRxSamples,
                "TimeoutCount": 0,
                "OverflowCount": 0,
                "LateCommandCount": 0,
                "OtherErrorCount": 0,
                "ScheduledHardwareTimeSec": float(Scheduled[Index]),
                "FirstSampleHardwareTimeSec": float(Scheduled[Index]),
                "TransmitQueued": False,
            }
            for Index in range(self.Timing.PulsesPerCpi)
        ]
        return SimpleNamespace(
            IQ=np.zeros(
                (self.Timing.PulsesPerCpi, self.Timing.NumRxSamples),
                dtype=np.complex64,
            ),
            SampleRate=self.Timing.SampleRateHz,
            PulseValid=np.ones(self.Timing.PulsesPerCpi, dtype=bool),
            Diagnostics={
                "ReceiveOnly": True,
                "OperatingMode": "RECEIVE_ONLY",
                "TimedTransmitEnabled": False,
                "SoftwareIqInjectionEnabled": False,
                "SBandStylePerPriLoop": True,
                "AdaptiveIndividualPriPipeline": False,
                "BoundedIndividualPriPipeline": True,
                "ReceiveCommandCount": self.Timing.PulsesPerCpi,
                "ConfiguredCommandQueueDepth": 20,
                "ActiveCommandQueueDepth": self.Timing.PulsesPerCpi,
                "CommandQueueHorizonSec": (
                    self.Timing.PulsesPerCpi * self.Timing.PriSec
                ),
                "MaximumOutstandingReceiveCommands": (
                    self.Timing.PulsesPerCpi
                ),
                "HardwareTimeQueriesPerDwell": 1,
                "ScheduledRxHardwareTimesSec": Scheduled,
                "PulseDiagnostics": PulseDiagnostics,
                "CaptureElapsedSec": 0.052,
            },
        )

    def test_default_arguments_are_conservative_and_atr_is_disabled(self):
        ValidateArguments(self.Arguments)
        self.assertEqual(self.Arguments.lead_ms, 50.0)
        self.assertFalse(self.Arguments.enable_atr_gpio)
        self.assertFalse(self.Config["EttusAtrGpioEnabled"])

    def test_receive_only_plan_explicitly_disables_every_transmit(self):
        self.assertTrue(all(
            not Pulse.TxEnabled
            for Pulse in self.Plan.PulsePlans
        ))
        self.assertFalse(self.Plan.Metadata["TimedTransmitPermitted"])

    def test_complete_error_free_dwell_passes(self):
        Raw = self.MakeRaw()
        Result = EvaluateHardwareDwell(Raw, self.Plan, self.Timing)

        self.assertTrue(Result.Passed)
        self.assertEqual(Result.Failures, [])
        self.assertEqual(
            Result.ReceivedSamples,
            self.Timing.PulsesPerCpi * self.Timing.NumRxSamples,
        )
        self.assertEqual(Result.MaximumTimestampErrorSec, 0.0)
        self.assertEqual(Result.MinimumSignedTimestampErrorSec, 0.0)
        self.assertEqual(Result.MeanSignedTimestampErrorSec, 0.0)
        self.assertEqual(Result.MaximumSignedTimestampErrorSec, 0.0)

    def test_uhd_error_or_tx_declaration_fails(self):
        Raw = self.MakeRaw()
        Raw.Diagnostics["TimedTransmitEnabled"] = True
        Raw.Diagnostics["PulseDiagnostics"][1]["TimeoutCount"] = 1
        Raw.Diagnostics["PulseDiagnostics"][1]["ReceivedSamples"] = 0
        Raw.PulseValid[1] = False

        Result = EvaluateHardwareDwell(Raw, self.Plan, self.Timing)

        self.assertFalse(Result.Passed)
        FailureText = "\n".join(Result.Failures)
        self.assertIn("TimedTransmitEnabled=False", FailureText)
        self.assertIn("timeouts", FailureText)
        self.assertIn("Invalid receive pulses", FailureText)

    def test_short_initial_command_lead_is_rejected(self):
        Arguments = ParseArguments(["--lead-ms", "5"])
        with self.assertRaisesRegex(ValueError, "at least 20 ms"):
            ValidateArguments(Arguments)


if __name__ == "__main__":
    unittest.main(verbosity=2)
