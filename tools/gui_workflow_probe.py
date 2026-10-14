"""Exercise the GUI workflow against USB CDC and save native Qt previews.

Uses actual button handlers, readback, telemetry, calibration progress/result.
Always shuts down the worker, which transmits the prioritized stop transaction.
"""
import json
import sys
import time
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'HostUI'))
from PyQt5 import QtCore, QtGui, QtWidgets
from crowerfoc_gui import MainWindow

app = QtWidgets.QApplication([])
app.setStyle('Fusion')
app.setFont(QtGui.QFont('Microsoft YaHei UI', 10))
w = MainWindow()
w.setAttribute(QtCore.Qt.WA_DontShowOnScreen)
w.show()
out = Path(__file__).resolve().parent
report = {}

def until(predicate, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        app.processEvents()
        if predicate():
            return
        time.sleep(.01)
    raise TimeoutError('GUI workflow timed out')

def preview(name):
    app.processEvents()
    w.grab().save(str(out / name))

try:
    w.toggle_conn()
    until(w._ready, 8)
    if w.worker.latest['fault']:
        w.clear_faults()
        until(lambda: not w.worker.latest['fault'], 3)
    w.sp_limit_cur.setValue(1)
    w.sp_limit_vel.setValue(10)
    w.bt_limits.click()
    until(lambda: not w._writes, 5)
    assert w.param_cache[0x0103] == 1
    assert 0 < w.param_cache[0x0104] <= 5
    assert not w.target_group.isEnabled()
    report['idle'] = dict(w.worker.latest)
    preview('gui_idle_layout.png')
    w.bt_cal.click()
    until(lambda: w._cal_seen, 5)
    assert not w.bt_closed.isEnabled() and not w.target_group.isEnabled()
    until(lambda: w.progress.value() >= 35, 18)
    report['progress'] = w.progress.value()
    report['progress_text'] = w.lb_progress.text()
    preview('gui_calibration_layout.png')
    print('PROGRESS', report['progress'], report['progress_text'], flush=True)
    until(lambda: bool(w._dialogs), 35)
    dialog = w._dialogs[0]
    report['result_title'] = dialog.windowTitle()
    report['result'] = dict(w._result_data)
    report['final'] = dict(w.worker.latest)
    assert dialog.windowTitle() == '完整标定结果'
    assert report['final']['state'] == 1 and report['final']['fault'] == 0
    assert report['final']['flags'] & 4
    assert 1 <= report['result'][0x0100] <= 64
    dialog.grab().save(str(out / 'gui_result_layout.png'))
    report['frames'] = w.worker.total
    report['missed'] = w.worker.missed
    print('RESULT', json.dumps(report, default=float, ensure_ascii=False), flush=True)
finally:
    w.close()
    until(lambda: w.worker is None, 4)
    app.processEvents()
    report['disconnected'] = True
    (out / 'gui_workflow_result.json').write_text(json.dumps(report, indent=2, ensure_ascii=False, default=float), encoding='utf-8')
