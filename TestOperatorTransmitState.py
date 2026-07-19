"""Hardware-free regression checks for receive-only operator state."""

import ast
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parent
DISPLAY_PATH = ROOT / "RadarDisplayQt5.py"
MAIN_PATH = ROOT / "VanguardxMain_scheduler.py"


class TestOperatorTransmitState(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.display_text = DISPLAY_PATH.read_text(encoding="utf-8")
        cls.main_text = MAIN_PATH.read_text(encoding="utf-8")

        # Parsing provides a hardware- and Qt-independent syntax check.
        ast.parse(cls.display_text, filename=str(DISPLAY_PATH))
        main_tree = ast.parse(cls.main_text, filename=str(MAIN_PATH))
        function_node = next(
            node
            for node in main_tree.body
            if isinstance(node, ast.FunctionDef)
            and node.name == "ApplyOperatorPointingCommand"
        )
        function_module = ast.fix_missing_locations(
            ast.Module(body=[function_node], type_ignores=[])
        )
        namespace = {}
        exec(compile(function_module, str(MAIN_PATH), "exec"), namespace)
        cls.apply_pointing_command = staticmethod(
            namespace["ApplyOperatorPointingCommand"]
        )

    def test_receive_only_status_is_explicit(self):
        self.assertIn('TransmitStatus = "INHIBITED (RX ONLY)"', self.display_text)
        self.assertIn('f"Tx:     {TransmitStatus}"', self.display_text)

    def test_tx_button_is_unavailable_without_rf_capability(self):
        self.assertIn('StopTxButton.setText("TX N/A")', self.display_text)
        self.assertIn("StopTxButton.setEnabled(False)", self.display_text)

    def test_start_does_not_unconditionally_enable_transmit(self):
        start_method = self.display_text.split(
            "    def OnStartScan(self):", 1
        )[1].split("    def OnStop(self):", 1)[0]
        self.assertIn("if self.TransmitAvailable:", start_method)
        self.assertNotIn(
            "self.ScanEnabled = True\n        self.TransmitEnabled = True",
            start_method,
        )

    def test_stop_disarms_transmit(self):
        stop_method = self.display_text.split(
            "    def OnStop(self):", 1
        )[1].split("    def OnStopTransmit(self):", 1)[0]
        self.assertIn("self.TransmitEnabled = False", stop_method)

    def test_main_allows_rx_dwells_while_tx_is_inhibited(self):
        self.assertIn(
            "TransmitEnabled or ReceiveOnlyOperation",
            self.main_text,
        )
        self.assertIn(
            'if DisplayMode == "STOP" or not RadarDwellEnabled:',
            self.main_text,
        )

    def test_stare_is_an_explicit_guarded_operating_command(self):
        stare_method = self.display_text.split(
            "    def OnStartStare(self):", 1
        )[1].split("    def OnStop(self):", 1)[0]
        self.assertIn('self.DisplayMode = "STARE"', stare_method)
        self.assertIn("self.ScanEnabled = False", stare_method)
        self.assertIn("if self.TransmitAvailable:", stare_method)
        self.assertIn("self.TransmitEnabled = True", stare_method)

    def test_nudge_buttons_use_visible_step_and_arm_guarded_operation(self):
        left_method = self.display_text.split(
            "    def OnBeamLeft(self):", 1
        )[1].split("    def OnBeamRight(self):", 1)[0]
        right_method = self.display_text.split(
            "    def OnBeamRight(self):", 1
        )[1].split("    def OnScanStartChanged(self):", 1)[0]

        for method in (left_method, right_method):
            self.assertIn("self.ScanStepDeg", method)
            self.assertNotIn("self.ManualBeamStepDeg", method)
            self.assertIn("if self.TransmitAvailable:", method)
            self.assertIn("self.TransmitEnabled = True", method)
            self.assertIn("self.ManualNudgeCommandId += 1", method)

        self.assertIn("-abs(float(self.ScanStepDeg))", left_method)
        self.assertIn("abs(float(self.ScanStepDeg))", right_method)

    def test_pointing_commands_run_before_radar_dwell_gate(self):
        loop_text = self.main_text.split("while not ExitRequested:", 1)[1]
        command_index = loop_text.index(
            ") = ApplyOperatorPointingCommand("
        )
        transition_consumed_index = loop_text.index(
            "LastDisplayMode = str(DisplayMode)"
        )
        dwell_gate_index = loop_text.index(
            'if DisplayMode == "STOP" or not RadarDwellEnabled:'
        )
        self.assertLess(command_index, dwell_gate_index)
        self.assertLess(transition_consumed_index, dwell_gate_index)

    def test_nudge_is_consumed_without_any_transmit_state(self):
        pointing = _FakePointing()
        command_id, action, target = self.apply_pointing_command(
            Pointing=pointing,
            ControlState={
                "ManualNudgeCommandId": 1,
                "ManualNudgeDeltaDeg": 2.0,
                "TransmitEnabled": False,
            },
            DisplayMode="STARE",
            ScanEnabled=False,
            LastDisplayMode="STOP",
            LastScanEnabled=False,
            LastManualNudgeCommandId=0,
        )
        self.assertEqual(command_id, 1)
        self.assertEqual(action, "NUDGE")
        self.assertEqual(target, 102.0)
        self.assertEqual(pointing.nudges, [2.0])

    def test_stare_transition_stops_scan_once(self):
        pointing = _FakePointing()
        command_id, action, target = self.apply_pointing_command(
            Pointing=pointing,
            ControlState={"ManualNudgeCommandId": 0},
            DisplayMode="STARE",
            ScanEnabled=False,
            LastDisplayMode="SCAN",
            LastScanEnabled=True,
            LastManualNudgeCommandId=0,
        )
        self.assertEqual(command_id, 0)
        self.assertEqual(action, "HOLD")
        self.assertIsNone(target)
        self.assertEqual(pointing.stop_count, 1)


class _FakePointing:
    def __init__(self):
        self.nudges = []
        self.stop_count = 0

    def Nudge(self, delta_deg):
        self.nudges.append(float(delta_deg))
        return 100.0 + float(delta_deg)

    def Stop(self):
        self.stop_count += 1


if __name__ == "__main__":
    unittest.main()
