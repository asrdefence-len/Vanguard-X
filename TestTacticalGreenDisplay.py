import pathlib
import unittest


class TacticalGreenDisplaySourceTests(unittest.TestCase):
    """GUI-free checks for the operational display colour hierarchy."""

    @classmethod
    def setUpClass(cls):
        source_dir = pathlib.Path(__file__).resolve().parent
        cls.radar_source = (source_dir / "RadarDisplayQt5.py").read_text(
            encoding="utf-8"
        )
        cls.simple_source = (source_dir / "SimpleDisplay.py").read_text(
            encoding="utf-8"
        )

    def test_qt_display_uses_configurable_tactical_green_framework(self):
        self.assertIn('Config.get("TacticalGreenColour", "#00d060")', self.radar_source)
        self.assertIn('Config.get("TacticalGridColour", "#176b3a")', self.radar_source)
        self.assertIn("ApplyTacticalPlotTheme(self.PpiPlot)", self.radar_source)
        self.assertIn("ApplyTacticalPlotTheme(self.RangePlot)", self.radar_source)

    def test_qt_range_rings_and_sector_boundaries_use_green_palette(self):
        self.assertIn(
            'Config.get("RangeRingColour", self.TacticalGridColour)',
            self.radar_source,
        )
        self.assertIn(
            'Config.get("SectorBoundaryColour", "#00a85a")',
            self.radar_source,
        )

    def test_track_identity_colours_are_preserved(self):
        self.assertIn(
            'Config.get("TentativeTrackPenColour", (255, 225, 100, 135))',
            self.radar_source,
        )
        self.assertIn(
            'Config.get("ConfirmedTrackPenColour", (255, 255, 255, 235))',
            self.radar_source,
        )

    def test_fallback_display_axes_are_green(self):
        self.assertIn("tick_params(colors=TacticalGreen)", self.simple_source)
        self.assertIn(
            'spines["polar"].set_color(TacticalGreen)',
            self.simple_source,
        )


if __name__ == "__main__":
    unittest.main()
