"""Bounded USB motion regression, 1 A, <=60 rpm and <=20 degree steps."""
import json,sys,time,math,hashlib
from pathlib import Path
from datetime import datetime,timezone,timedelta
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'HostUI'))
from PyQt5 import QtWidgets
from crowerfoc_client import USBClient
from crowerfoc_protocol import *
ROOT=Path(__file__).resolve().parents[1]
app=QtWidgets.QApplication([])
progress=QtWidgets.QProgressDialog('连接','停止',0,100);progress.setWindowTitle('位置与输入模式回归 · 1A');progress.setAutoClose(False);progress.setMinimumDuration(0);progress.show()
axis=None;result={'units':'deg,rpm,A','status':'failed','checks':{}}
stamp=datetime.now(timezone(timedelta(hours=8))).strftime('%Y%m%d_%H%M%S')
try:
    axis=USBClient(progress);axis.stop();axis.capture(.3,False)
    if not axis.worker.latest['flags']&4 or axis.worker.latest['fault']:raise RuntimeError('缺有效校准或存在故障')
    ids=(0x0103,0x0400,0x0402,0x0300,0x0301,0x0302,0x0305,0x0500,0x0501,0x0310,0x0311,0x0312)
    previous={p:axis.get(p) for p in ids};result['previous']={f'{p:04X}':v for p,v in previous.items()}
    axis.set(0x0103,1);axis.set(0x0400,60)
    axis.set(0x0300,5);axis.set(0x0301,.15/RPM_PER_RAD_S);axis.set(0x0302,.8/RPM_PER_RAD_S)
    axis.set(0x0310,15);axis.set(0x0311,30);axis.set(0x0312,30)
    for index,input_mode in enumerate((1,3,4)):
        axis.stop();axis.capture(.6,False);axis.set(0x0500,2);axis.set(0x0501,input_mode)
        for direction in (1,-1):
            progress.setValue(5+index*20+(8 if direction==-1 else 0));progress.setLabelText(f'位置模式{input_mode} · {direction*20:+d}°')
            axis.state(3);axis.capture(.1)
            target=axis.worker.latest['pos']+direction*20;axis.point(target)
            rows=axis.capture(7)
            tail=[r for r in rows if r['state']==3][-100:]
            error=abs(float(np.mean([r['pos'] for r in tail]))-target)
            current=max(abs(r['iq'])/1000 for r in rows if r['state']==3)
            speed=max(abs(r['vel']) for r in rows if r['state']==3)
            result['checks'][f'position_{input_mode}_{direction}']={'target_deg':target,'error_deg':error,'max_iq_A':current,'max_rpm':speed}
            print(f'mode={input_mode} direction={direction} error_deg={error:.3f} peak_rpm={speed:.2f}',flush=True)
            if error>2:raise RuntimeError(f'位置误差{error:.2f}°仍偏大')
            axis.stop();axis.capture(.8,False)
    progress.setValue(72);progress.setLabelText('速度斜坡与电流斜坡')
    axis.set(0x0500,1);axis.set(0x0501,2);axis.set(0x0305,30);axis.state(3);axis.point(30);axis.capture(3)
    result['checks']['velocity_ramp_final_rpm']=axis.worker.latest['vel'];axis.stop();axis.capture(.8,False)
    axis.set(0x0500,0);axis.set(0x0501,5);axis.set(0x0306,.1);axis.state(3);axis.point(.1);rows=axis.capture(.6)
    peak=max(abs(r.get('iq_target_A',0)) for r in rows if r['state']==3)
    result['checks']['torque_ramp_0_6s_target_A']=peak
    if peak>.08:raise RuntimeError('电流斜坡变化率不符')
    axis.stop();axis.capture(.8,False)
    progress.setValue(88);progress.setLabelText('配置有效组合原子导入，随后重新校准')
    old=axis.get(0x0430)
    assert axis.command(MSG_CONFIG_PATCH,b'\x00')==0
    import struct
    assert axis.command(MSG_CONFIG_PATCH,b'\x01'+struct.pack('<Hf',0x0430,old))==0
    assert axis.command(MSG_CONFIG_PATCH,b'\x02')==0
    axis.capture(.1,False);assert not axis.worker.latest['flags']&4
    result['checks']['atomic_valid_restore_invalidates_calibration']=True
    # Restore test changes before a full calibration and final save.
    for p in (0x0305,0x0310,0x0311,0x0312,0x0400,0x0402):axis.set(p,previous[p])
    axis.set(0x0500,0);axis.set(0x0501,1);axis.set(0x0103,40)
    axis.state(2);began=time.monotonic()
    while time.monotonic()-began<23:
        x=axis.read()
        if x and x['fault']:raise RuntimeError('恢复校准失败')
        if x and x['state']==1 and time.monotonic()-began>1:
            if not x['flags']&4:raise RuntimeError('恢复校准无效')
            break
    else:raise TimeoutError('恢复校准超时')
    if axis.command(MSG_SAVE):raise RuntimeError('保存失败')
    result['checks']['configured_position_gain']=axis.get(0x0300)
    result['checks']['configured_velocity_gain_A_per_rpm']=axis.get(0x0301)
    result['checks']['configured_velocity_integrator_A_per_rpm_s']=axis.get(0x0302)
    result['checks']['isr_max_us']=axis.get(0x061A)/144
    result['status']='passed'
except Exception as exc:result['error']=str(exc);print('FAIL',exc,flush=True)
finally:
    if axis:
        result['samples']=axis.samples
        try:axis.stop();axis.capture(.15,False)
        except Exception:pass
        # Any failed test retains a 1 A limit until reviewed; never raises on error.
        try:axis.close()
        except Exception:pass
    folder=ROOT/'tools/axis_feature_results';folder.mkdir(exist_ok=True);path=folder/f'{stamp}_control.json'
    from axis_extensions import configuration_document
    def plain(x):
        if isinstance(x,dict):return {str(k):plain(v) for k,v in x.items()}
        if isinstance(x,(list,tuple)):return [plain(v) for v in x]
        if isinstance(x,(float,np.floating)) and not math.isfinite(x):return None
        if isinstance(x,np.generic):return x.item()
        return x
    path.write_text(json.dumps(plain(result),ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
    progress.close();dialog=QtWidgets.QMessageBox();dialog.setWindowTitle('控制回归结果');dialog.setText(result['status']+' '+result.get('error',''));dialog.setInformativeText(str(path));dialog.show();app.processEvents()
    print(json.dumps({'path':str(path),'status':result['status'],'checks':result['checks'],'error':result.get('error')},ensure_ascii=False),flush=True)
if result['status']!='passed':sys.exit(1)
