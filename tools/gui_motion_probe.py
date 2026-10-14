"""Actual GUI mode/readback/start/target/stop/active-close checks."""
import json
import sys
import time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'HostUI'))
from PyQt5 import QtCore, QtGui, QtWidgets
from crowerfoc_gui import MainWindow
app=QtWidgets.QApplication([])
app.setFont(QtGui.QFont('Microsoft YaHei UI',10))
w=MainWindow(); w.setAttribute(QtCore.Qt.WA_DontShowOnScreen); w.show()
report={}; samples=[]; last_seq=None

def until(check,seconds):
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        app.processEvents()
        if check(): return
        time.sleep(.005)
    raise TimeoutError('GUI condition timed out')

try:
    w.toggle_conn(); until(w._ready,8)
    assert w.worker.latest['flags']&4
    for pid,value in ((0x0301,.15),(0x0302,.2)):
        w.do_set(pid,value); until(lambda:not w._writes,4)
        assert abs(w.param_cache[pid]-value)<1e-6
    w.sp_limit_vel.setValue(6); w.sp_limit_cur.setValue(1)
    w.bt_limits.click(); until(lambda:not w._writes,4)
    w.cb_mode.setCurrentIndex(1)
    until(lambda:w.param_cache.get(0x0500)==1 and not w._writes,4)
    w.bt_closed.click()
    until(lambda:w.target_group.isEnabled() and w.worker.latest['state']==3,4)
    assert w.sp_set.value()==0
    w.sp_set.setValue(1); w.bt_set.click(); until(lambda:not w._writes,4)
    end=time.monotonic()+4
    while time.monotonic()<end:
        app.processEvents(); x=w.worker.latest
        if x['seq']!=last_seq:
            samples.append({k:float(v) if hasattr(v,'item') else v for k,v in x.items()})
            last_seq=x['seq']
        assert not x['fault'] and abs(x['vel'])<8
        time.sleep(.005)
    report['running']=dict(w.worker.latest)
    w.grab().save(str(Path(__file__).with_name('gui_running_layout.png')))
    worker=w.worker
    w.close(); until(lambda:w.worker is None,4)
    report['closed_final']=dict(worker.latest)
    assert worker.latest['state']==1 and not worker.latest['fault']
    w2=MainWindow(); w2.setAttribute(QtCore.Qt.WA_DontShowOnScreen); w2.show()
    w2.toggle_conn(); until(w2._ready,8)
    report['reconnected']=dict(w2.worker.latest)
    assert w2.worker.latest['state']==1 and not w2.target_group.isEnabled()
    w2.close(); until(lambda:w2.worker is None,4)
    report['passed']=True
finally:
    if w.worker:
        w.close(); until(lambda:w.worker is None,4)
    report['samples']=samples
    Path(__file__).with_name('gui_motion_result.json').write_text(json.dumps(report,indent=2,ensure_ascii=False,default=float),encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k!='samples'},default=float,ensure_ascii=False),flush=True)
