"""Stage 2B integration tests for derived radar timing and RX windows."""

from types import SimpleNamespace
import unittest

import numpy as np

from DataTypes import RawDwellData
from EttusRadarSource import EttusRadarSource
from RadarExecutor import RadarExecutor
from RadarPlans import make_uniform_dwell_plan
from RadarProcessor import RadarProcessor
from RadarTasks import RadarTaskType
from RadarTiming import CalculateRadarTiming, SPEED_OF_LIGHT_MPS
from SimulatedSource import SimulatedSource
from WaveformLibrary import WaveformLibrary


def MakeBaseConfig():
    return {
        "EttusSampleRateHz": 40.0e6,
        "EttusRxWarmupEnabled": False,
        "RfFrequency": 9.4e9,
        "SearchWaveformId": "Frank10_20MHz",
        "TrackWaveformId": "Barker13_20MHz",
        "MinPrfHz": 1000.0,
        "MaxPrfHz": 4000.0,
        "SelectedPrfHz": 2000.0,
        "SelectedPulsesPerCpi": 32,
        "InstrumentedMaxRangeM": 15000.0,
        "ReceiverRecoveryTimeSec": 1.0e-6,
        "RxEndMarginSec": 2.0e-6,
        "NextTxGuardTimeSec": 2.0e-6,
        "PTZScanSlewRateDegPerSec": 14.0,
        "TransmitPowerW": 50.0,
        "AntennaGainDb": 23.0,
        "SystemLossDb": 6.0,
        "NoisePowerW": 0.0,
    }


def MakeLibrary(Config):
    Library = WaveformLibrary(Config)
    Library.LoadDefaultWaveforms()
    return Library


def MakePlan(Timing, DwellId=1):
    return make_uniform_dwell_plan(
        dwell_id=DwellId,
        task_id=1,
        task_type="SEARCH",
        waveform_id=Timing.WaveformId,
        sample_rate=Timing.SampleRateHz,
        num_samples=Timing.NumRxSamples,
        num_pulses=Timing.PulsesPerCpi,
        pri_sec=Timing.PriSec,
        rx_start_delay_sec=Timing.RxStartDelaySec,
        metadata={"RadarTiming": Timing.ToMetadata()},
    )


class FakeHardwareTime:
    def __init__(self, ValueSec):
        self.ValueSec = float(ValueSec)

    def get_real_secs(self):
        return self.ValueSec


class FakeUsrp:
    def __init__(self):
        self.TimeQueryCount = 0

    def get_time_now(self):
        self.TimeQueryCount += 1
        return FakeHardwareTime(100.0)

    def get_rx_freq(self, Channel):
        return 1.0e9

    def get_rx_gain(self, Channel):
        return 10.0

    def get_rx_antenna(self, Channel):
        return "RX2"


class RecordingEttusSource(EttusRadarSource):
    """Test double that records timed commands without requiring UHD."""

    def __init__(self, Config, Library):
        super().__init__(Config, Library)
        self.Usrp = FakeUsrp()
        self.IssuedCommands = []
        self.Events = []
        self._initialised = True

    def _configure_sample_rate(self, requested_rate):
        return float(requested_rate)

    def _issue_receive_command(self, num_samples, scheduled_time_sec):
        self.Events.append("ARM_RX")
        self.IssuedCommands.append(
            (int(num_samples), float(scheduled_time_sec))
        )

    def _queue_transmit_for_pri(
        self,
        ThisDwell,
        pulse_index,
        scheduled_pri_time_sec,
        tx_time_offset_sec=None,
        rf_target=None,
    ):
        self.Events.append("TX_HOOK_DISABLED")
        return super()._queue_transmit_for_pri(
            ThisDwell,
            pulse_index,
            scheduled_pri_time_sec,
            tx_time_offset_sec=tx_time_offset_sec,
            rf_target=rf_target,
        )

    def _receive_scheduled_pri(self, num_samples):
        self.Events.append("COLLECT_RX")
        return (
            np.zeros(int(num_samples), dtype=np.complex64),
            {
                "RequestedSamples": int(num_samples),
                "ReceivedSamples": int(num_samples),
                "ReceiveCalls": 1,
                "TimeoutCount": 0,
                "OverflowCount": 0,
                "LateCommandCount": 0,
                "OtherErrorCount": 0,
                "LastMetadataError": "none",
                "FirstSampleHardwareTimeSec": None,
            },
        )


class TestRadarTimingIntegration(unittest.TestCase):
    def test_existing_positional_interfaces_remain_compatible(self):
        Config = MakeBaseConfig()
        Library = MakeLibrary(Config)
        Executor = RadarExecutor(
            SimpleNamespace(TheWaveformLibrary=Library),
            None,
            Config,
            True,
        )
        self.assertTrue(Executor.Debug)

        Plan = make_uniform_dwell_plan(
            1,
            2,
            "SEARCH",
            "Frank10_20MHz",
            40.0e6,
            128,
            4,
            500.0e-6,
            12.0,
            3.0,
            4.0,
            5.0,
            {"LegacyPositionalCall": True},
        )
        self.assertEqual(Plan.AzimuthDeg, 12.0)
        self.assertEqual(Plan.RxStartDelaySec, 0.0)

        Raw = RawDwellData(
            1,
            np.zeros((4, 128), dtype=np.complex64),
            40.0e6,
            500.0e-6,
            0.0,
            np.arange(4, dtype=np.float64) * 500.0e-6,
            np.full(4, 500.0e-6),
            ["Frank10_20MHz"] * 4,
            np.ones(4, dtype=bool),
            {"LegacyPositionalCall": True},
        )
        self.assertTrue(Raw.Diagnostics["LegacyPositionalCall"])
        self.assertIsNone(Raw.PulseRxStartDelaySec)

    def test_executor_profiles_are_derived_from_waveform_and_operator_inputs(self):
        Config = MakeBaseConfig()
        Library = MakeLibrary(Config)
        Source = SimpleNamespace(TheWaveformLibrary=Library)
        Executor = RadarExecutor(
            source=Source,
            pointing_manager=None,
            config=Config,
            waveform_library=Library,
        )

        Search = Executor.GetExecutionProfile(
            SimpleNamespace(TaskType=RadarTaskType.SEARCH)
        )
        Track = Executor.GetExecutionProfile(
            SimpleNamespace(TaskType=RadarTaskType.TRACK)
        )

        self.assertEqual(Search.WaveformId, "Frank10_20MHz")
        self.assertEqual(Search.NumSamples, 4043)
        self.assertEqual(Search.NumPulses, 32)
        self.assertAlmostEqual(Search.PriSec, 500.0e-6, places=15)
        self.assertAlmostEqual(Search.RxStartDelaySec, 6.2e-6, places=15)
        self.assertAlmostEqual(
            Search.Timing.AntennaMovementDuringCpiDeg,
            0.224,
            places=12,
        )

        self.assertEqual(Track.WaveformId, "Barker13_20MHz")
        self.assertEqual(Track.NumSamples, 4043)
        self.assertAlmostEqual(Track.RxStartDelaySec, 1.85e-6, places=15)
        self.assertEqual(Track.Timing.AntennaMovementDuringCpiDeg, 0.0)

    def test_pulse_plan_carries_derived_receive_window(self):
        Config = MakeBaseConfig()
        Library = MakeLibrary(Config)
        Timing = CalculateRadarTiming(
            Library.GetMetadata("Frank10_20MHz")
        )
        Plan = MakePlan(Timing)

        self.assertEqual(Plan.NumSamples, 4043)
        self.assertEqual(Plan.NumPulses, 32)
        self.assertAlmostEqual(Plan.PRI, 500.0e-6, places=15)
        self.assertAlmostEqual(Plan.RxStartDelaySec, 6.2e-6, places=15)
        self.assertTrue(all(
            pulse.NumRxSamples == 4043
            for pulse in Plan.PulsePlans
        ))
        self.assertTrue(all(
            abs(pulse.RxStartDelaySec - 6.2e-6) < 1e-15
            for pulse in Plan.PulsePlans
        ))

    def test_simulator_and_processor_preserve_absolute_target_range(self):
        Config = MakeBaseConfig()
        TargetRangeM = 8000.0
        Config["SceneReturns"] = [
            {
                "name": "TimingTarget",
                "range_m": TargetRangeM,
                "doppler_hz": 0.0,
                "rcs": 1000.0,
                "two_way_gain_power": 1.0,
            }
        ]
        Library = MakeLibrary(Config)
        Timing = CalculateRadarTiming(
            Library.GetMetadata("Frank10_20MHz"),
            PulsesPerCpi=8,
        )
        Plan = MakePlan(Timing)

        Source = SimulatedSource(Config, Library)
        Raw = Source.ExecuteDwell(Plan)
        Processor = RadarProcessor(Config, Library)
        Processed = Processor.Process(Raw, Plan)

        RangeBinWidthM = SPEED_OF_LIGHT_MPS / (2.0 * 40.0e6)
        self.assertEqual(Raw.IQ.shape, (8, 4043))
        np.testing.assert_allclose(
            Raw.PulseRxStartDelaySec,
            6.0e-6,
            rtol=0.0,
            atol=1e-15,
        )
        self.assertAlmostEqual(
            Processed.RangeAxisM[0],
            Timing.FirstRxSampleRangeOffsetM,
            places=9,
        )
        self.assertLessEqual(
            abs(Processed.Diagnostics["PeakRangeM"] - TargetRangeM),
            RangeBinWidthM / 2.0 + 1e-9,
        )
        self.assertAlmostEqual(
            Processed.Diagnostics["RxStartDelaySec"],
            6.0e-6,
            places=15,
        )

    def test_ettus_receive_commands_include_per_pulse_rx_delay(self):
        Config = MakeBaseConfig()
        Config["EttusCommandLeadTimeSec"] = 0.005
        Config["EttusAtrGpioEnabled"] = False
        Config["EttusTxLeadingZeroSamples"] = 8
        Library = MakeLibrary(Config)
        Timing = CalculateRadarTiming(
            Library.GetMetadata("Frank10_20MHz"),
            PulsesPerCpi=4,
        )
        Plan = MakePlan(Timing)
        Source = RecordingEttusSource(Config, Library)

        Raw = Source.ExecuteDwell(Plan)

        self.assertEqual(Raw.IQ.shape, (4, 4043))
        self.assertEqual(len(Source.IssuedCommands), 4)
        ExpectedFirstRxTimeSec = 100.0 + 0.005 + 6.2e-6
        self.assertAlmostEqual(
            Source.IssuedCommands[0][1],
            ExpectedFirstRxTimeSec,
            places=12,
        )
        for PulseIndex, (NumSamples, ScheduledTimeSec) in enumerate(
            Source.IssuedCommands
        ):
            self.assertEqual(NumSamples, 4043)
            self.assertAlmostEqual(
                ScheduledTimeSec,
                ExpectedFirstRxTimeSec + PulseIndex * 500.0e-6,
                places=12,
            )
        np.testing.assert_allclose(
            Raw.PulseRxStartDelaySec,
            6.0e-6,
            rtol=0.0,
            atol=1e-15,
        )
        self.assertTrue(
            Raw.Diagnostics["AllReceiveCommandsQueuedBeforeCollection"]
        )
        self.assertTrue(Raw.Diagnostics["SBandStylePerPriLoop"])
        self.assertTrue(Raw.Diagnostics["BoundedIndividualPriPipeline"])
        self.assertEqual(Raw.Diagnostics["ReceiveCommandCount"], 4)
        self.assertEqual(Raw.Diagnostics["ConfiguredCommandQueueDepth"], 20)
        self.assertEqual(Raw.Diagnostics["ActiveCommandQueueDepth"], 4)
        self.assertEqual(
            Raw.Diagnostics["MaximumOutstandingReceiveCommands"],
            4,
        )
        self.assertEqual(Raw.Diagnostics["HardwareTimeQueriesPerDwell"], 1)
        self.assertEqual(Source.Usrp.TimeQueryCount, 1)
        self.assertEqual(
            Source.Events,
            [
                "TX_HOOK_DISABLED", "ARM_RX",
                "TX_HOOK_DISABLED", "ARM_RX",
                "TX_HOOK_DISABLED", "ARM_RX",
                "TX_HOOK_DISABLED", "ARM_RX",
                "COLLECT_RX",
                "COLLECT_RX",
                "COLLECT_RX",
                "COLLECT_RX",
            ],
        )
        self.assertFalse(Raw.Diagnostics["TimedTransmitEnabled"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
