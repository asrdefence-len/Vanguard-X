import math
import unittest

from RadarMapOverlay import RadarCentredMap


class TestNswRegionalMap(unittest.TestCase):
    def setUp(self):
        self.map = RadarCentredMap(
            "NSWCoast_Newcastle_to_BatemansBay_OSM_20260722.json",
            -34.368,
            150.929,
        )

    def test_multi_geometry_loads(self):
        self.assertGreater(len(self.map.ProjectCoastlines()), 10)
        self.assertGreater(len(self.map.ProjectLandPolygons()), 10)

    def test_bellambi_coast_is_near_radar(self):
        nearest = min(
            math.hypot(east, north)
            for line in self.map.ProjectCoastlines()
            for east, north in line
        )
        self.assertLess(nearest, 2000.0)

    def test_moving_centre_changes_projection(self):
        before = self.map.ProjectCoordinate(150.929, -34.368)
        self.map.SetRadarPosition(-34.500, 150.900)
        after = self.map.ProjectCoordinate(150.929, -34.368)
        self.assertLess(math.hypot(*before), 1.0)
        self.assertGreater(math.hypot(*after), 10000.0)

    def test_labels_filter_to_display_range(self):
        names = {row[0] for row in self.map.ProjectLabels(15000.0)}
        self.assertIn("Bellambi", names)
        self.assertNotIn("Sydney", names)


if __name__ == "__main__":
    unittest.main()
