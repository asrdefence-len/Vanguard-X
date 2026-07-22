import json
import math
import os
import tempfile
import unittest

from RadarMapOverlay import LocalEastNorthM, RadarCentredMap


class TestRadarMapOverlay(unittest.TestCase):
    def setUp(self):
        self.MapPath = os.path.join(os.path.dirname(__file__), "NSWCoast_Newcastle_to_BatemansBay_OSM_20260722.json")

    def test_radar_centre_projects_to_origin(self):
        east_m, north_m = LocalEastNorthM(-34.368, 150.929, -34.368, 150.929)
        self.assertAlmostEqual(east_m, 0.0)
        self.assertAlmostEqual(north_m, 0.0)

    def test_north_and_east_have_correct_ppi_sign(self):
        radar_map = RadarCentredMap(self.MapPath, -34.368, 150.929)
        east_m, north_m = radar_map.ProjectCoordinate(150.939, -34.358)
        self.assertGreater(east_m, 0.0)
        self.assertGreater(north_m, 0.0)

    def test_labels_are_filtered_to_display_range(self):
        radar_map = RadarCentredMap(self.MapPath, -34.368, 150.929)
        near_names = {record[0] for record in radar_map.ProjectLabels(5000.0)}
        self.assertIn("Bellambi", near_names)
        self.assertNotIn("Port Kembla", near_names)

    def test_position_update_moves_map_under_fixed_radar(self):
        radar_map = RadarCentredMap(self.MapPath, -34.368, 150.929)
        original = radar_map.ProjectCoastline()[0]
        radar_map.SetRadarPosition(-34.358, 150.939)
        moved = radar_map.ProjectCoastline()[0]
        self.assertLess(moved[0], original[0])
        self.assertLess(moved[1], original[1])


if __name__ == "__main__":
    unittest.main()
