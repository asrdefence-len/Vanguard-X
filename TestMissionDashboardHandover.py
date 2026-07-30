#!/usr/bin/env python3
"""Regression tests for Mission-to-Dashboard control handover."""

import ast
from pathlib import Path
import types
import unittest


ROOT = Path(__file__).resolve().parent
DISPLAY_PATH = ROOT / "RadarDisplayQt5.py"


def _LoadSetMissionRuntimeStatus():
    tree = ast.parse(
        DISPLAY_PATH.read_text(encoding="utf-8"),
        filename=str(DISPLAY_PATH),
    )
    class_node = next(
        node
        for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == "RadarDisplay"
    )
    method_node = next(
        node
        for node in class_node.body
        if isinstance(node, ast.FunctionDef)
        and node.name == "SetMissionRuntimeStatus"
    )
    module = ast.fix_missing_locations(
        ast.Module(body=[method_node], type_ignores=[])
    )
    namespace = {}
    exec(compile(module, str(DISPLAY_PATH), "exec"), namespace)
    return namespace["SetMissionRuntimeStatus"]


class MissionDashboardHandoverTests(unittest.TestCase):
    def _Display(self):
        display = types.SimpleNamespace(
            MissionPage=None,
            LastMissionRuntimeStatusRevision=-1,
            DisplayMode="SCAN",
            ScanEnabled=True,
            TransmitAvailable=True,
            TransmitEnabled=True,
        )
        display.UpdateStatusPanel = lambda: None
        return display

    def test_terminal_transition_stops_once_but_repeat_does_not_reclaim(self):
        apply_status = _LoadSetMissionRuntimeStatus()
        display = self._Display()
        aborted = {
            "StatusRevision": 7,
            "State": "ABORTED",
            "Message": "Mission aborted by operator",
        }

        apply_status(display, aborted)
        self.assertEqual(display.DisplayMode, "STOP")
        self.assertFalse(display.ScanEnabled)
        self.assertFalse(display.TransmitEnabled)

        # Simulate a later, explicit Dashboard Start.
        display.DisplayMode = "SCAN"
        display.ScanEnabled = True
        display.TransmitEnabled = True

        # The scheduler republishes status every dwell. The unchanged Mission
        # revision must update the page text without cancelling Dashboard mode.
        apply_status(display, aborted)
        self.assertEqual(display.DisplayMode, "SCAN")
        self.assertTrue(display.ScanEnabled)
        self.assertTrue(display.TransmitEnabled)

    def test_new_terminal_revision_still_forces_safe_stop(self):
        apply_status = _LoadSetMissionRuntimeStatus()
        display = self._Display()
        apply_status(display, {
            "StatusRevision": 7,
            "State": "ABORTED",
        })

        display.DisplayMode = "SCAN"
        display.ScanEnabled = True
        display.TransmitEnabled = True
        apply_status(display, {
            "StatusRevision": 8,
            "State": "FAULTED",
        })

        self.assertEqual(display.DisplayMode, "STOP")
        self.assertFalse(display.ScanEnabled)
        self.assertFalse(display.TransmitEnabled)


if __name__ == "__main__":
    unittest.main()
