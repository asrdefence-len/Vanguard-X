"""Tests for Vanguard X scan-based angular detection consolidation."""

from types import SimpleNamespace
import unittest

import numpy as np

from AngularDetectionProcessor import AngularDetectionProcessor
from DataTypes import Detection
from EarthReferencedMeasurements import AnnotateDetectionsWithEarthReference
from RadarRemoteDisplay import BuildDisplaySnapshot
from TargetScenario import antenna_gain_power_sinc


class AngularDetectionProcessorTests(unittest.TestCase):
    def setUp(self):
        self.config = {
            "AngularDetectionProcessingEnabled": True,
            "BeamwidthDeg": 5.0,
            "SidelobeFloorDb": -50.0,
            "RangeBinM": 15.0,
            "AngularPerDwellClusterRangeGapM": 100.0,
            "AngularPerDwellClusterDopplerGapHz": 120.0,
            "AngularAssociationRangeGateM": 250.0,
            "AngularAssociationDopplerGateHz": 180.0,
            "AngularMaximumSampleGapDeg": 4.0,
            "AngularCloseAfterMissedDwells": 2,
            "AngularCloseAfterMissedAngleDeg": 5.0,
            "AngularMaximumMissedDwells": 12,
            "AngularMinimumFitSamples": 3,
            "AngularHypothesisStepDeg": 0.05,
            "AngularPlotPersistenceSec": 60.0,
        }

    def _processed(self, bearing_deg, dwell_id, scan_cycle=0, task_id=1):
        return SimpleNamespace(
            DwellId=dwell_id,
            TimeStamp=0.1 * dwell_id,
            Diagnostics={
                "BoresightDeg": bearing_deg,
                "ScanCycle": scan_cycle,
                "ScheduledTaskId": task_id,
                "ScheduledTaskType": "SEARCH",
            },
        )

    def _detections_for_extended_target(
        self,
        beam_bearing_deg,
        dwell_id,
        target_bearing_deg=344.3,
        centre_range_m=5280.0,
        length_m=200.0,
        scatterers=9,
    ):
        cross_range_offsets_m = np.linspace(
            -0.5 * length_m,
            0.5 * length_m,
            scatterers,
        )
        target_bearings_deg = (
            target_bearing_deg
            + np.rad2deg(cross_range_offsets_m / centre_range_m)
        )
        ranges_m = centre_range_m + np.linspace(
            -0.45 * length_m,
            0.45 * length_m,
            scatterers,
        )
        detections = []
        for index, (bearing_deg, range_m) in enumerate(
            zip(target_bearings_deg, ranges_m)
        ):
            angle_error_deg = (
                (bearing_deg - beam_bearing_deg + 180.0) % 360.0
            ) - 180.0
            one_way = antenna_gain_power_sinc(
                angle_error_deg,
                5.0,
                -50.0,
            )
            two_way = float(one_way) ** 2
            power = 1.0e8 * two_way
            if power <= 3.0e3:
                continue
            detections.append(
                Detection(
                    DwellId=dwell_id,
                    RangeBin=int(round(range_m / 15.0)),
                    DopplerBin=16,
                    RangeM=float(range_m),
                    DopplerHz=60.0,
                    VelocityMps=1.0,
                    AzimuthDeg=float(beam_bearing_deg),
                    AmplitudeDb=float(10.0 * np.log10(power)),
                    TimeStamp=0.1 * dwell_id,
                )
            )
            detections[-1].SnrDb = float(
                10.0 * np.log10(power / 1.0e3)
            )
        return detections

    def _run_crossing(self, processor, target_bearing_deg=344.3):
        emitted = []
        bearings = [336.0, 338.0, 340.0, 342.0, 344.0, 346.0, 348.0, 350.0, 352.0]
        for dwell_id, beam_bearing_deg in enumerate(bearings):
            detections = self._detections_for_extended_target(
                beam_bearing_deg,
                dwell_id,
                target_bearing_deg=target_bearing_deg,
            )
            emitted.extend(
                processor.Update(
                    detections,
                    self._processed(beam_bearing_deg, dwell_id),
                )
            )
        for offset, beam_bearing_deg in enumerate([354.0, 356.0, 358.0]):
            emitted.extend(
                processor.Update(
                    [],
                    self._processed(
                        beam_bearing_deg,
                        len(bearings) + offset,
                    ),
                )
            )
        return emitted

    def test_high_snr_angular_smear_becomes_one_plot(self):
        processor = AngularDetectionProcessor(self.config)
        plots = self._run_crossing(processor)
        self.assertEqual(len(plots), 1)
        self.assertLess(
            abs(((plots[0].AzimuthDeg - 344.3 + 180.0) % 360.0) - 180.0),
            0.4,
        )
        self.assertGreaterEqual(plots[0].NumCfarHitDwells, 4)

    def test_broadside_vessel_keeps_range_extent_without_width_classification(self):
        processor = AngularDetectionProcessor(self.config)
        plot = self._run_crossing(processor)[0]
        self.assertGreater(plot.RangeExtentM, 150.0)
        self.assertFalse(hasattr(plot, "TargetWidthM"))
        self.assertEqual(plot.PatternModel, "BUILT_IN_TWO_WAY_SINC4")

    def test_raw_dwell_bearings_are_not_emitted_as_plots(self):
        processor = AngularDetectionProcessor(self.config)
        plot = self._run_crossing(processor)[0]
        self.assertEqual(len(processor.GetDisplayPlots()), 1)
        self.assertNotIn(
            round(plot.AzimuthDeg, 6),
            {336.0, 338.0, 340.0, 342.0, 344.0, 346.0, 348.0, 350.0, 352.0},
        )

    def test_crossing_continues_when_scan_cycle_changes_in_same_task(self):
        processor = AngularDetectionProcessor(self.config)
        for dwell_id, beam_bearing_deg in enumerate([340.0, 342.0, 344.0]):
            processor.Update(
                self._detections_for_extended_target(
                    beam_bearing_deg,
                    dwell_id,
                ),
                self._processed(beam_bearing_deg, dwell_id, scan_cycle=0),
            )
        plots = processor.Update(
            [],
            self._processed(346.0, 4, scan_cycle=1),
        )
        self.assertEqual(plots, [])
        self.assertEqual(len(processor.ActiveCrossings), 1)

        for dwell_id, beam_bearing_deg in enumerate(
            [348.0, 350.0, 352.0],
            start=5,
        ):
            plots.extend(
                processor.Update(
                    [],
                    self._processed(
                        beam_bearing_deg,
                        dwell_id,
                        scan_cycle=1,
                    ),
                )
            )
        self.assertEqual(len(plots), 1)

    def test_crossing_is_flushed_when_task_changes(self):
        processor = AngularDetectionProcessor(self.config)
        for dwell_id, beam_bearing_deg in enumerate([340.0, 342.0, 344.0]):
            processor.Update(
                self._detections_for_extended_target(
                    beam_bearing_deg,
                    dwell_id,
                ),
                self._processed(
                    beam_bearing_deg,
                    dwell_id,
                    task_id=1,
                ),
            )
        plots = processor.Update(
            [],
            self._processed(346.0, 4, task_id=2),
        )
        self.assertEqual(len(plots), 1)
        self.assertEqual(len(processor.ActiveCrossings), 0)

    def test_brief_cfar_gap_does_not_split_one_crossing(self):
        processor = AngularDetectionProcessor(self.config)
        emitted = []
        bearings = [338.0, 340.0, 342.0, 344.0, 346.0, 348.0, 350.0, 352.0]
        hit_bearings = {340.0, 342.0, 346.0, 348.0}
        for dwell_id, beam_bearing_deg in enumerate(bearings):
            detections = (
                self._detections_for_extended_target(
                    beam_bearing_deg,
                    dwell_id,
                )
                if beam_bearing_deg in hit_bearings
                else []
            )
            emitted.extend(
                processor.Update(
                    detections,
                    self._processed(beam_bearing_deg, dwell_id),
                )
            )

        for dwell_id, beam_bearing_deg in enumerate(
            [354.0, 356.0],
            start=len(bearings),
        ):
            emitted.extend(
                processor.Update(
                    [],
                    self._processed(beam_bearing_deg, dwell_id),
                )
            )

        self.assertEqual(len(emitted), 1)
        self.assertEqual(emitted[0].NumCfarHitDwells, 4)

    def test_sector_reversal_does_not_emit_two_edge_plots(self):
        processor = AngularDetectionProcessor(self.config)
        emitted = []
        samples = [
            (340.0, 0),
            (342.0, 0),
            (344.0, 0),
            (346.0, 0),
            (348.0, 1),
            (346.0, 1),
            (344.0, 1),
            (342.0, 1),
            (340.0, 1),
            (338.0, 1),
            (336.0, 1),
        ]
        for dwell_id, (beam_bearing_deg, scan_cycle) in enumerate(samples):
            detections = self._detections_for_extended_target(
                beam_bearing_deg,
                dwell_id,
                target_bearing_deg=345.0,
            )
            emitted.extend(
                processor.Update(
                    detections,
                    self._processed(
                        beam_bearing_deg,
                        dwell_id,
                        scan_cycle=scan_cycle,
                    ),
                )
            )
        emitted.extend(processor.Flush())
        self.assertEqual(len(emitted), 1)

    def test_wraparound_target_is_fitted_on_circular_bearing_axis(self):
        processor = AngularDetectionProcessor(self.config)
        emitted = []
        bearings = [352.0, 354.0, 356.0, 358.0, 0.0, 2.0, 4.0, 6.0]
        for dwell_id, beam_bearing_deg in enumerate(bearings):
            detections = self._detections_for_extended_target(
                beam_bearing_deg,
                dwell_id,
                target_bearing_deg=359.2,
            )
            emitted.extend(
                processor.Update(
                    detections,
                    self._processed(beam_bearing_deg, dwell_id),
                )
            )
        for offset, beam_bearing_deg in enumerate([8.0, 10.0, 12.0]):
            emitted.extend(
                processor.Update(
                    [],
                    self._processed(
                        beam_bearing_deg,
                        len(bearings) + offset,
                    ),
                )
            )
        self.assertEqual(len(emitted), 1)
        error_deg = abs(
            ((emitted[0].AzimuthDeg - 359.2 + 180.0) % 360.0) - 180.0
        )
        self.assertLess(error_deg, 0.5)

    def test_completed_plot_can_be_earth_referenced_and_sent_to_remote_ui(self):
        processor = AngularDetectionProcessor(self.config)
        plot = self._run_crossing(processor)[0]
        pose = SimpleNamespace(
            TimestampSec=10.0,
            SequenceNumber=4,
            PositionEnu=SimpleNamespace(
                east_m=5000.0,
                north_m=1000.0,
                up_m=0.0,
            ),
            VelocityEnu=SimpleNamespace(
                east_mps=0.0,
                north_mps=0.0,
                up_mps=0.0,
            ),
            HeadingTrueDeg=90.0,
            PositionValid=True,
            HeadingValid=True,
            VelocityValid=True,
            Source="TEST",
        )
        measurements = AnnotateDetectionsWithEarthReference([plot], pose)
        self.assertTrue(measurements[0].Valid)
        snapshot = BuildDisplaySnapshot(
            {},
            self._processed(350.0, 20),
            [],
            [],
            [plot],
        )
        wire_plot = snapshot["plots"][0]
        self.assertEqual(wire_plot["PlotId"], plot.PlotId)
        self.assertEqual(wire_plot["RangeExtentM"], plot.RangeExtentM)
        self.assertEqual(
            wire_plot["PatternModel"],
            "BUILT_IN_TWO_WAY_SINC4",
        )


if __name__ == "__main__":
    unittest.main()
