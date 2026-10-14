"""Render the real GUI offline with explicitly synthetic telemetry. No serial I/O."""
import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
import sys
import struct
import time
import json
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'HostUI'))
from PyQt5 import QtWidgets, QtGui
from crowerfoc_gui import MainWindow
from crowerfoc_transport import SerialWorker
from crowerfoc_protocol import MSG_TELEMETRY


def main():
    app = QtWidgets.QApplication([])
    app.setStyle('Fusion')
    # Qt offscreen does not enumerate Windows fonts; load one explicitly.
    font_id = QtGui.QFontDatabase.addApplicationFont('C:/Windows/Fonts/msyh.ttc')
    family = QtGui.QFontDatabase.applicationFontFamilies(font_id)
    app.setFont(QtGui.QFont(family[0] if family else 'Microsoft YaHei UI', 10))
    w = MainWindow()
    w.timer.stop()
    w.setWindowTitle('CrowerFOC 校准界面离线示意 · 非实测数据')
    r = SerialWorker('offline-preview')  # Never start its thread or open a port.
    w.worker, w._connected = r, True
    payload = bytearray(36)
    struct.pack_into('<H', payload, 22, 23100)
    payload[32], payload[33], payload[34], payload[35] = 2, 0, 1, 2
    r._dispatch(MSG_TELEMETRY, payload)
    for pid, value in {0x0100: 0, 0x0200: 0, 0x0201: 1,
                       0x0103: 1, 0x0104: 5, 0x0400: 6, 0x0402: 8,
                       0x0500: 0, 0x020C: 14.50015}.items():
        w.on_param_reply(pid, value)
    w._cal_started = time.monotonic()
    w.show()
    out = Path(__file__).resolve().parent
    for stage, progress, filename in ((0, 3, 'calibration_ui_step1_offline.png'),
                                      (3, 50, 'calibration_ui_step4_offline.png')):
        w.on_param_reply(0x0208, stage)
        w.on_param_reply(0x020B, progress)
        w._update_device()
        w.state_banner.setText('离线界面示意（非实测） · ' + w.state_banner.text())
        app.processEvents()
        w.grab().save(str(out / filename))
    result_file = sorted((out / 'current_loop_results').glob('*_final_auto_pi_torque.json'))[-1]
    measured = json.loads(result_file.read_text(encoding='utf-8'))['calibration']
    w._result_data = {int(pid, 16): value for pid, value in measured.items()}
    w.show_result(True)
    dialog = w._dialogs[-1]
    dialog.setWindowTitle('离线结果布局 · 数值来自实测记录')
    app.processEvents()
    dialog.grab().save(str(out / 'calibration_result_auto_pi_offline.png'))
    dialog.close()
    w.worker = None
    w.close()
    app.processEvents()


if __name__ == '__main__':
    main()
