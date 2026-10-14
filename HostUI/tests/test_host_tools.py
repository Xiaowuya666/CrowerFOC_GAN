import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
import json
import tempfile
import unittest
import zipfile
from pathlib import Path

from PyQt5 import QtWidgets

ROOT = Path(__file__).resolve().parents[1]
import sys
sys.path.insert(0, str(ROOT))
from host_tools import create_diagnostic_bundle, CanDebugWidget
from diagnostic_mode import ComponentChecklistDialog
from crowerfoc_transport import SerialWorker
from crowerfoc_protocol import MSG_SET
from mit_can import unpack_control
from crowerfoc_gui import MainWindow

APP = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


class HostToolTests(unittest.TestCase):
    def test_diagnostic_bundle_contains_device_diagnostics_without_recordings(self):
        with tempfile.TemporaryDirectory() as tmp:
            zip_path = Path(tmp) / "diagnostics.zip"
            create_diagnostic_bundle(zip_path, {
                "metadata": {"host_ui": "test"}, "status": {"state": 1},
                "parameters": {"0103": 1}, "logs": "test log",
                "hardware_scenarios": [{"reference": "R1"}],
                "fault_history": [{"fault": 4}],
                "samples": [{"seq": 1}], "high_speed_capture": [{"ia_A": 0.2}],
            })
            with zipfile.ZipFile(zip_path) as bundle:
                names = bundle.namelist()
                self.assertNotIn("遥测记录.csv", names)
                self.assertNotIn("高速捕获.json", names)
                self.assertIn("网表器件与失效情景.json", names)
                self.assertEqual(json.loads(bundle.read("故障历史.json")), [{"fault": 4}])
                self.assertEqual(json.loads(bundle.read("设备状态.json")), {"state": 1})

    def test_can_frame_builder_and_component_checklist(self):
        can = CanDebugWidget()
        can.pos.setValue(180)
        can.vel.setValue(60)
        can.iq.setValue(2.0)
        can.make_control()
        self.assertIn("DLC=8", can.frame.text())
        wire = bytes.fromhex(can.frame.text().split("DATA=")[1])
        self.assertEqual(len(wire), 8)
        decoded = unpack_control(wire)
        self.assertAlmostEqual(decoded[0], 3.14159, delta=0.001)
        self.assertAlmostEqual(decoded[1], 6.28318, delta=0.02)
        checklist = ComponentChecklistDialog()
        self.assertGreaterEqual(len(checklist.parts), 75)
        checklist.state.setCurrentText("异常")
        checklist.notes.setPlainText("虚焊待复查")
        checklist.list.setCurrentRow(1)
        self.assertEqual(checklist.results["主板/" + checklist.parts[0][1]]["status"], "异常")
        can.close()
        checklist.close()

    def test_main_window_has_separate_workspaces_and_faster_redraw(self):
        window = MainWindow()
        labels = [window.workspaces.tabText(i) for i in range(window.workspaces.count())]
        self.assertEqual(labels, ["控制与校准", "运行曲线", "参数与配置", "故障与自检", "CAN调试"])
        self.assertEqual(window.timer.interval(), 20)
        self.assertFalse(any("记录器" in window.workspaces.tabText(i)
                             for i in range(window.workspaces.count())))
        window.timer.stop()
        window.close()


if __name__ == "__main__":
    unittest.main()
