import pathlib
import unittest


class CompactDashboardStatusSourceTests(unittest.TestCase):
    """GUI-free checks for the compact dashboard status panel."""

    @classmethod
    def setUpClass(cls):
        source_dir = pathlib.Path(__file__).resolve().parent
        cls.radar_source = (source_dir / "RadarDisplayQt5.py").read_text(
            encoding="utf-8"
        )

    def test_confirmed_and_tentative_counts_share_one_line(self):
        self.assertIn(
            'f"Tracks: {NumConfirmed}, Tent: {NumTentative}"',
            self.radar_source,
        )
        self.assertNotIn('f"Tent:   {NumTentative}"', self.radar_source)

    def test_detection_and_plot_counts_share_one_line(self):
        self.assertIn(
            'f"Dets: {NumDetections} "',
            self.radar_source,
        )
        self.assertIn(
            'f"Plots: {NumPlots} "',
            self.radar_source,
        )
        self.assertNotIn(
            'f"Dets:   {NumDetections} ',
            self.radar_source,
        )
        self.assertNotIn(
            'f"Plots:  {NumPlots} ',
            self.radar_source,
        )

    def test_peak_range_and_velocity_share_one_line(self):
        peak_range = (
            'f"Peak R: {Diagnostics.get(\'PeakRangeM\', 0.0):.1f} m, "'
        )
        peak_velocity = (
            'f"Peak V: {Diagnostics.get(\'PeakVelocityMps\', 0.0):.1f} m/s"'
        )
        self.assertIn(peak_range, self.radar_source)
        self.assertIn(peak_velocity, self.radar_source)
        self.assertNotIn(f"Lines.append({peak_range}", self.radar_source)
        self.assertNotIn(f"Lines.append({peak_velocity}", self.radar_source)

    def test_redundant_status_rows_are_removed(self):
        self.assertNotIn(
            'f"X6-60:   {self.X660AzimuthRelativeDeg:.1f} deg rel"',
            self.radar_source,
        )
        self.assertNotIn(
            'f"Y-axis: {self.RangeProfileDisplayMinDb:.1f} to "',
            self.radar_source,
        )


if __name__ == "__main__":
    unittest.main()
