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
        ast.parse(cls.main_text, filename=str(MAIN_PATH))

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

    def test_main_allows_rx_dwells_while_tx_is_inhibited(self):
        self.assertIn(
            "TransmitEnabled or ReceiveOnlyOperation",
            self.main_text,
        )
        self.assertIn(
            'if DisplayMode == "STOP" or not RadarDwellEnabled:',
            self.main_text,
        )


if __name__ == "__main__":
    unittest.main()
