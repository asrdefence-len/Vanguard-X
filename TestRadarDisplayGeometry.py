"""Regression tests for the Vanguard X North-up PPI bearing convention."""

import importlib.util
import sys
import types
import unittest


# The geometry helper is static and does not require a GUI.  Permit this test
# to run on headless CI hosts that do not install PyQtGraph/PyQt5.
if importlib.util.find_spec("pyqtgraph") is None:
    sys.modules["pyqtgraph"] = types.ModuleType("pyqtgraph")

if importlib.util.find_spec("PyQt5") is None:
    PyQt5 = types.ModuleType("PyQt5")
    PyQt5.__path__ = []
    PyQt5.QtCore = types.ModuleType("PyQt5.QtCore")
    PyQt5.QtGui = types.ModuleType("PyQt5.QtGui")
    PyQt5.QtWidgets = types.ModuleType("PyQt5.QtWidgets")
    sys.modules["PyQt5"] = PyQt5
    sys.modules["PyQt5.QtCore"] = PyQt5.QtCore
    sys.modules["PyQt5.QtGui"] = PyQt5.QtGui
    sys.modules["PyQt5.QtWidgets"] = PyQt5.QtWidgets

from RadarDisplayQt5 import RadarDisplay


class TestRadarDisplayGeometry(unittest.TestCase):
    def assert_xy(self, bearing_deg, expected_x, expected_y):
        x, y = RadarDisplay.AngleRangeToXY(bearing_deg, 1000.0)
        self.assertAlmostEqual(x, expected_x, places=9)
        self.assertAlmostEqual(y, expected_y, places=9)

    def test_cardinal_bearings_are_north_up_and_clockwise_positive(self):
        self.assert_xy(0.0, 0.0, 1000.0)       # North: top
        self.assert_xy(90.0, 1000.0, 0.0)      # East: right
        self.assert_xy(180.0, 0.0, -1000.0)    # South: bottom
        self.assert_xy(270.0, -1000.0, 0.0)    # West: left

    def test_bearing_wrap_preserves_north(self):
        self.assert_xy(360.0, 0.0, 1000.0)

    def test_clockwise_quadrants_follow_radar_bearing_convention(self):
        north_east = 1000.0 / (2.0 ** 0.5)
        self.assert_xy(45.0, north_east, north_east)
        self.assert_xy(135.0, north_east, -north_east)


if __name__ == "__main__":
    unittest.main(verbosity=2)
