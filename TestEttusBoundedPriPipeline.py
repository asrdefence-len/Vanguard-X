"""Regression tests for the fixed-depth individual-PRI pipeline."""

from types import SimpleNamespace
import unittest

import numpy as np

from EttusRadarSource import EttusRadarSource
from RadarPlans import make_uniform_dwell_plan
from RadarTiming import CalculateRadarTiming
from WaveformLibrary import WaveformLibrary


def MakeConfig():
    return {
        "EttusSampleRateHz": 40.0e6,
        "EttusCommandLeadTimeSec": 0.050,
        "EttusRxWarmupEnabled": False,
        "EttusAtrGpioEnabled": False,
        "RfFrequency": 9.4e9,
    }


def MakePlan(Timing):
    return make_uniform_dwell_plan(
        dwell_id=1,
        task_id=1,
        task_type="SEARCH",
        waveform_id=Timing.WaveformId,
        sample_rate=Timing.SampleRateHz,
        num_samples=Timing.NumRxSamples,
        num_pulses=Timing.PulsesPerCpi,
        pri_sec=Timing.PriSec,
        rx_start_delay_sec=Timing.RxStartDelaySec,
    )


class FakeHardwareTime:
    def get_real_secs(self):
        return 100.0


class OneQueryUsrp:
    def __init__(self):
        self.TimeQueryCount = 0

    def get_time_now(self):
        self.TimeQueryCount += 1
        return FakeHardwareTime()

    def get_rx_freq(self, Channel):
        return 1.0e9

    def get_rx_gain(self, Channel):
        return 10.0

    def get_rx_antenna(self, Channel):
        return "RX2"


class BoundedPipelineSource(EttusRadarSource):
    def __init__(self, Config, Library):
        super().__init__(Config, Library)
        self.Usrp = OneQueryUsrp()
        self.Events = []
        self.NextArmIndex = 0
        self.NextCollectIndex = 0
        self._initialised = True

    def _configure_sample_rate(self, requested_rate):
        return float(requested_rate)

    def _warm_up_receive_path(self, num_samples):
        self.Events.append(("WARMUP_RX", int(num_samples)))
        return {
            "RequestedSamples": int(num_samples),
            "ReceivedSamples": int(num_samples),
            "TimeoutCount": 0,
            "OverflowCount": 0,
            "LateCommandCount": 0,
            "BrokenChainCount": 0,
            "AlignmentErrorCount": 0,
            "BadPacketCount": 0,
            "OtherErrorCount": 0,
            "LastMetadataError": "none",
        }

    def _queue_transmit_for_pri(
        self,
        ThisDwell,
        pulse_index,
        scheduled_pri_time_sec,
        tx_time_offset_sec=None,
        rf_target=None,
    ):
        self.Events.append(
            ("TX_HOOK_DISABLED", pulse_index, scheduled_pri_time_sec)
        )
        return False

    def _issue_receive_command(self, num_samples, scheduled_time_sec):
        PulseIndex = self.NextArmIndex
        self.NextArmIndex += 1
        self.Events.append(
            (
                "ARM_RX",
                PulseIndex,
                int(num_samples),
                float(scheduled_time_sec),
            )
        )

    def _receive_scheduled_pri(self, num_samples):
        PulseIndex = self.NextCollectIndex
        self.NextCollectIndex += 1
        self.Events.append(("COLLECT_RX", PulseIndex, int(num_samples)))
        return np.zeros(int(num_samples), dtype=np.complex64), {
            "RequestedSamples": int(num_samples),
            "ReceivedSamples": int(num_samples),
            "ReceiveCalls": 1,
            "TimeoutCount": 0,
            "OverflowCount": 0,
            "LateCommandCount": 0,
            "BrokenChainCount": 0,
            "AlignmentErrorCount": 0,
            "BadPacketCount": 0,
            "OtherErrorCount": 0,
            "LastMetadataError": "none",
            "LastMetadataErrorCodeValue": 0,
            "LastMetadataErrorText": "",
            "MetadataErrorEvents": [],
            "FirstSampleHardwareTimeSec": None,
        }


class TestEttusBoundedPriPipeline(unittest.TestCase):
    def setUp(self):
        self.Config = MakeConfig()
        self.Library = WaveformLibrary(self.Config)
        self.Library.LoadDefaultWaveforms()

    def Execute(self, PrfHz, Pulses=128):
        Timing = CalculateRadarTiming(
            self.Library.GetMetadata("Frank10_20MHz"),
            SelectedPrfHz=float(PrfHz),
            PulsesPerCpi=int(Pulses),
        )
        Source = BoundedPipelineSource(self.Config, self.Library)
        Raw = Source.ExecuteDwell(MakePlan(Timing))
        return Timing, Source, Raw

    def test_2khz_maintains_twenty_individual_finite_pris(self):
        Timing, Source, Raw = self.Execute(2000.0)

        self.assertEqual(Source.Usrp.TimeQueryCount, 1)
        self.assertEqual(Raw.Diagnostics["HardwareTimeQueriesPerDwell"], 1)
        self.assertEqual(Raw.Diagnostics["ReceiveCommandCount"], 128)
        self.assertEqual(
            Raw.Diagnostics["MaximumOutstandingReceiveCommands"],
            20,
        )
        self.assertTrue(Raw.Diagnostics["SBandStylePerPriLoop"])
        self.assertTrue(Raw.Diagnostics["BoundedIndividualPriPipeline"])
        self.assertEqual(Raw.Diagnostics["ConfiguredCommandQueueDepth"], 20)
        self.assertEqual(Raw.Diagnostics["ActiveCommandQueueDepth"], 20)
        self.assertTrue(np.all(Raw.PulseValid))
        self.assertEqual(Raw.IQ.shape, (128, 4043))

        for PulseIndex in range(128):
            TxPosition = next(
                Index for Index, Event in enumerate(Source.Events)
                if Event[0] == "TX_HOOK_DISABLED"
                and Event[1] == PulseIndex
            )
            ArmPosition = next(
                Index for Index, Event in enumerate(Source.Events)
                if Event[0] == "ARM_RX" and Event[1] == PulseIndex
            )
            CollectPosition = next(
                Index for Index, Event in enumerate(Source.Events)
                if Event[0] == "COLLECT_RX" and Event[1] == PulseIndex
            )
            self.assertLess(TxPosition, ArmPosition)
            self.assertLess(ArmPosition, CollectPosition)

        TxTimes = np.asarray([
            Event[2]
            for Event in Source.Events
            if Event[0] == "TX_HOOK_DISABLED"
        ])
        RxTimes = np.asarray([
            Event[3]
            for Event in Source.Events
            if Event[0] == "ARM_RX"
        ])
        np.testing.assert_allclose(
            np.diff(TxTimes),
            Timing.PriSec,
            rtol=0.0,
            atol=1.0e-12,
        )
        np.testing.assert_allclose(
            RxTimes - TxTimes,
            Timing.RxStartDelaySec,
            rtol=0.0,
            atol=1.0e-12,
        )

    def test_depth_is_twenty_across_operator_prf_range(self):
        for PrfHz in (
            1000.0,
            2000.0,
            2500.0,
            4000.0,
        ):
            with self.subTest(PrfHz=PrfHz):
                Timing, Source, Raw = self.Execute(PrfHz)

                np.testing.assert_allclose(
                    np.diff(
                        Raw.Diagnostics["ScheduledPriHardwareTimesSec"]
                    ),
                    Timing.PriSec,
                    rtol=0.0,
                    atol=1.0e-12,
                )
                self.assertEqual(Source.Usrp.TimeQueryCount, 1)
                self.assertEqual(
                    Raw.Diagnostics["ConfiguredCommandQueueDepth"],
                    20,
                )
                self.assertEqual(
                    Raw.Diagnostics["ActiveCommandQueueDepth"],
                    20,
                )
                self.assertEqual(
                    Raw.Diagnostics[
                        "MaximumOutstandingReceiveCommands"
                    ],
                    20,
                )
                self.assertAlmostEqual(
                    Raw.Diagnostics["CommandQueueHorizonSec"],
                    20 * Timing.PriSec,
                    places=15,
                )
                self.assertFalse(
                    Raw.Diagnostics[
                        "AllReceiveCommandsQueuedBeforeCollection"
                    ]
                )

    def test_metadata_error_codes_are_reported_by_numeric_uhd_value(self):
        self.assertEqual(EttusRadarSource._error_name(2), "late_command")
        self.assertEqual(EttusRadarSource._error_name(4), "broken_chain")
        self.assertEqual(EttusRadarSource._error_name(12), "alignment")
        self.assertEqual(EttusRadarSource._error_name(15), "bad_packet")

    def test_receive_stream_warmup_occurs_once_before_first_dwell_t0(self):
        Config = dict(self.Config)
        Config["EttusRxWarmupEnabled"] = True
        Timing = CalculateRadarTiming(
            self.Library.GetMetadata("Frank10_20MHz"),
            SelectedPrfHz=2000.0,
            PulsesPerCpi=4,
        )
        Source = BoundedPipelineSource(Config, self.Library)

        First = Source.ExecuteDwell(MakePlan(Timing))
        Second = Source.ExecuteDwell(MakePlan(Timing))

        self.assertTrue(First.Diagnostics["RxWarmupPerformed"])
        self.assertFalse(Second.Diagnostics["RxWarmupPerformed"])
        self.assertEqual(
            len([Event for Event in Source.Events if Event[0] == "WARMUP_RX"]),
            1,
        )
        self.assertEqual(Source.Usrp.TimeQueryCount, 2)


if __name__ == "__main__":
    unittest.main(verbosity=2)
