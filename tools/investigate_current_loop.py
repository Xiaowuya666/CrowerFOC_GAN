"""Bounded motor diagnosis, per-frame raw/conditioned capture, stop on every exit.
No motor R/L/pole count is supplied by this test. Calibration measures them.
"""
import argparse
import json
import math
import struct
import subprocess
import sys
import time
from datetime import datetime, timezone, timedelta
from pathlib import Path
import numpy as np
import serial
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'HostUI'))
from PyQt5 import QtWidgets
from crowerfoc_transport import SerialWorker, find_port
from crowerfoc_protocol import *


class Recorder(SerialWorker):
    def __init__(self, port):
        super().__init__(port)
        self.samples = []
        self.started = time.monotonic()
        self.segment = 'connect'
        self.stream_hz = 100

    def _on_telem(self, payload):
        super()._on_telem(payload)
        s = dict(self.latest)
        s.update(t=time.monotonic()-self.started, segment=self.segment, stream_hz=self.stream_hz)
        index = (self.count-1) % self.n
        for k in ('da', 'db', 'dc'):
            s[k] = float(self.data[k][index])
        self.samples.append(s)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--label', default='baseline_protected')
    parser.add_argument('--mode', choices=('baseline', 'calibrate', 'velocity', 'lifecycle'), default='baseline')
    parser.add_argument('--vel-kp', type=float, default=.15)
    parser.add_argument('--vel-ki', type=float, default=.2)
    parser.add_argument('--targets', help='Explicit comma-separated test targets')
    parser.add_argument('--speed-guard', type=float, default=6.5)
    parser.add_argument('--actual-speed-limit', type=float)
    parser.add_argument('--pulse-seconds', type=float, default=.04)
    parser.add_argument('--stream-hz', type=int, choices=(100,500,1000))
    parser.add_argument('--current-kp', type=float)
    parser.add_argument('--current-ki', type=float)
    args = parser.parse_args()
    app = QtWidgets.QApplication([])
    app.setStyle('Fusion')
    progress = QtWidgets.QProgressDialog('连接并检查空闲状态', '立即停止', 0, 100)
    progress.setWindowTitle('CrowerFOC 电流环定位 · 短时转动测试')
    progress.setMinimumDuration(0)
    progress.setAutoClose(False)
    progress.setAutoReset(False)
    progress.resize(590, 160)
    progress.show()
    app.processEvents()
    port = find_port()
    if not port:
        raise RuntimeError('电机 USB 不在线')
    r = Recorder(port)
    params, trace = {}, {}
    r.param.connect(lambda k,v: params.update({k:v}))
    r.current_trace.connect(lambda k,v: trace.update({k:list(v)}))
    buf = bytearray()
    sp = serial.Serial(port, 115200, timeout=.005, write_timeout=.2)
    next_hb = 0
    heartbeat_enabled = True
    checks = []
    stamp_zone = timezone(timedelta(hours=8))
    result = dict(label=args.label, time=datetime.now(stamp_zone).isoformat(),
                  mode=args.mode, cases=checks, commands=[], samples=r.samples)
    def send(t,p=b''):
        result['commands'].append(dict(t=time.monotonic()-r.started, type=t, payload=p.hex()))
        sp.write(encode_frame(t,p))
    def read(guard=True):
        nonlocal next_hb
        app.processEvents()
        if progress.wasCanceled() and guard:
            raise InterruptedError('用户停止')
        now = time.monotonic()
        if heartbeat_enabled and now >= next_hb:
            send(MSG_HEARTBEAT)
            next_hb=now+.15
        before=len(r.samples)
        buf.extend(sp.read(max(1,min(sp.in_waiting,8192))))
        r._parse(buf)
        if guard:
            for s in r.samples[before:]:
                if s['fault']:
                    raise RuntimeError('设备故障 '+str(s))
                if s['state'] != 1:
                    cap = 6500 if s['state']==2 else 3000
                    speed_cap = 6.5 if s['state']==2 else args.speed_guard
                    if max(abs(s[k]) for k in ('ia','ib','ic','id','iq')) > cap or abs(s['vel']) > speed_cap or not 16000<s['vbus']<26000:
                        raise RuntimeError('主机诊断保护 '+str(s))
        if guard and r.last_telem and time.monotonic()-r.last_telem > .35:
            raise RuntimeError('遥测中断')
    def capture(seconds,guard=True):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            read(guard)
    def get(pid, guard=True):
        params.pop(pid,None)
        send(MSG_GET,struct.pack('<H',pid))
        end=time.monotonic()+.5
        while pid not in params and time.monotonic()<end:
            read(guard)
        if pid not in params:
            raise RuntimeError(f'参数{pid:04x}超时')
        return params[pid]
    def setp(pid,value):
        send(MSG_SET,struct.pack('<Hf',pid,value))
        actual=get(pid)
        if not math.isclose(actual,value,rel_tol=1e-5,abs_tol=1e-5):
            raise RuntimeError(f'写入拒绝{pid:04x}: {value} -> {actual}')
        if pid==0x0630:
            r.stream_hz=value
    def stop():
        sp.write(stop_packet())
        capture(.15,False)
        if not r.latest or r.latest['state']!=1:
            raise RuntimeError('停止后未确认空闲')
    def diagnostics():
        return {hex(k):get(k,False) for k in (0x0100,0x0101,0x0102,0x0103,0x0104,0x0200,0x0201,0x0202,0x0207,0x020A,0x020D,0x020E,0x020F,*range(0x0210,0x0215),0x0303,0x0304,0x0402,*range(0x0600,0x0605),*range(0x0610,0x061B),*range(0x0620,0x0625))}
    def wait_stationary():
        r.segment='coast'
        end=time.monotonic()+12
        stable=0
        seen=-1
        while time.monotonic()<end:
            read()
            if r.latest and r.latest['seq']!=seen:
                seen=r.latest['seq']
                stable=stable+1 if abs(r.latest['vel'])<.6 else 0
                if stable>=30:
                    return
        raise RuntimeError('轴未自行停稳；结束本轮')
    def fetch_trace():
        if get(0x061D,False)==3:
            for i in range(64):
                send(MSG_GET,struct.pack('<H',0x0700+i))
                end=time.monotonic()+.35
                while i not in trace and time.monotonic()<end:
                    read(False)
            return dict(trigger=get(0x061E,False),next=get(0x061F,False),records=dict(trace))
        return None
    try:
        stop()
        capture(.4)
        if not r.latest or not r.latest['flags']&2 or not 16000<r.latest['vbus']<26000:
            raise RuntimeError('母线/编码器不正常')
        result['before']=diagnostics()
        if args.stream_hz:
            result['stream_hz_before']=get(0x0630)
            setp(0x0630,args.stream_hz)
            result['encoder_config']={hex(k):get(k) for k in range(0x0210,0x0215)}
        for pid,value in ((0x0303,args.current_kp),(0x0304,args.current_ki)):
            if value is not None:
                setp(pid,value)
        if args.actual_speed_limit is not None:
            setp(0x0402,args.actual_speed_limit)
        if args.mode=='calibrate' or not r.latest['flags']&4:
            r.segment='calibration'
            send(MSG_STATE,b'\x02')
            started=time.monotonic()
            stage=-1
            while time.monotonic()-started<22:
                capture(.15)
                p=get(0x020B)
                ph=get(0x0208)
                progress.setValue(int(p*.45))
                progress.setLabelText(f'完整标定：步骤 {int(ph)+1}/6，{p:.0f}%')
                if ph!=stage:
                    print('CAL',ph,p,flush=True)
                    stage=ph
                if r.latest['state']==1 and r.latest['flags']&4:
                    break
            if not r.latest['flags']&4:
                raise RuntimeError('标定未完成')
            result['calibration']=diagnostics()
            print('CALIBRATION',json.dumps(result['calibration']),flush=True)
            box=QtWidgets.QMessageBox(QtWidgets.QMessageBox.Information,'标定结果',
                '\n'.join(f'{k}: {result["calibration"][k]}' for k in ('0x100','0x101','0x102','0x200','0x201','0x20a')),parent=progress)
            box.setModal(False)
            box.show()
            app.processEvents()
        if args.mode=='lifecycle':
            wait_stationary()
            r.segment='debug_attach_detach'
            progress.setLabelText('低速运行中检查 DAP 连接/断开与 USB 心跳')
            progress.setValue(55)
            setp(0x0103,1.2)
            setp(0x0301,.15)
            setp(0x0302,.2)
            setp(0x0500,1)
            send(MSG_STATE,b'\x03')
            send(MSG_SETPOINT,struct.pack('<f',2.0))
            capture(1)
            debug_log=Path(__file__).resolve().parent/'fault_results/debug_lifecycle_output.txt'
            with debug_log.open('w',encoding='utf-8') as log:
                process=subprocess.Popen([str(Path(__file__).resolve().parents[1]/'.tools/pyocd-venv/Scripts/python.exe'),
                                          str(Path(__file__).resolve().parent/'read_device_swd.py')],stdout=log,stderr=log)
                deadline=time.monotonic()+8
                while process.poll() is None and time.monotonic()<deadline:
                    capture(.05)
                if process.poll() is None:
                    process.terminate()
                    raise RuntimeError('DAP lifecycle inspection timeout')
                if process.returncode:
                    raise RuntimeError('DAP lifecycle inspection failed')
            capture(1)
            if r.latest['state']!=3 or r.latest['fault']:
                raise RuntimeError('DAP disconnect interrupted drive')
            result['debug_lifecycle']=dict(r.latest)
            print('DAP_LIFECYCLE_OK',flush=True)
            progress.setLabelText('关闭主机心跳，验证固件自行关断')
            progress.setValue(80)
            r.segment='heartbeat_loss'
            heartbeat_enabled=False
            lost_at=time.monotonic()
            while time.monotonic()-lost_at<1.3:
                read(False)
                if r.latest['fault']:
                    break
            result['heartbeat_loss']=dict(delay_s=time.monotonic()-lost_at,sample=dict(r.latest))
            if r.latest['fault']!=512 or r.latest['state']!=1:
                raise RuntimeError('通信看门狗未按预期停机')
            print('HEARTBEAT_STOP_OK',result['heartbeat_loss']['delay_s'],flush=True)
        elif args.mode!='calibrate':
            if args.mode=='velocity':
                setp(0x0301,args.vel_kp)
                setp(0x0302,args.vel_ki)
                targets=(1,-1,2,-2,4,-4,6,-6)
            else:
                targets=(.4,-.4,.8,-.8,1,-1)
            if args.targets:
                targets=tuple(float(v) for v in args.targets.split(','))
            if args.mode=='velocity':
                setp(0x0400,max(10,max(abs(v) for v in targets)))
            for index,target in enumerate(targets):
                stop()
                wait_stationary()
                name=f'{args.mode}_{target:+g}'
                r.segment=name
                progress.setLabelText(f'{name}：同步记录原始电流、d/q、机械角和占空比')
                progress.setValue(45+int(48*index/len(targets)))
                setp(0x0103,1.2)
                setp(0x0500,1 if args.mode=='velocity' else 0)
                begin=len(r.samples)
                send(MSG_STATE,b'\x03')
                capture(.05)
                for fraction in (.25,.5,.75,1):
                    send(MSG_SETPOINT,struct.pack('<f',target*fraction))
                    capture(.1 if args.mode=='velocity' else .01)
                capture(3 if args.mode=='velocity' else args.pulse_seconds)
                stop()
                checks.append(dict(name=name,start=begin,end=len(r.samples),diagnostics=diagnostics(),trace=fetch_trace()))
                print('CASE',name, json.dumps(checks[-1]['diagnostics']),flush=True)
        stop()
        result['after']=diagnostics()
        progress.setValue(100)
        result['completed']=True
    except Exception as exc:
        result['error']=str(exc)
        print('ERROR',exc,flush=True)
    finally:
        try:
            stop()
            result['final']=dict(r.latest)
            result['after']=diagnostics()
            result['fault_trace']=fetch_trace()
            if 'before' in result:
                for pid in (0x0103,0x0402)+tuple(pid for pid,value in ((0x0303,args.current_kp),(0x0304,args.current_ki)) if value is not None):
                    send(MSG_SET,struct.pack('<Hf',pid,result['before'][hex(pid)]))
                    result.setdefault('restored',{})[hex(pid)]=get(pid,False)
            if 'stream_hz_before' in result:
                send(MSG_SET,struct.pack('<Hf',0x0630,result['stream_hz_before']))
                result.setdefault('restored',{})['0x630']=get(0x0630,False)
            send(MSG_SET,struct.pack('<Hf',0x0400,10))
            send(MSG_SET,struct.pack('<Hf',0x0500,0))
            result.setdefault('restored',{})['0x400']=get(0x0400,False)
        except Exception as exc:
            result['cleanup_error']=str(exc)
        sp.close()
        out=Path(__file__).resolve().parent/'current_loop_results'
        out.mkdir(exist_ok=True)
        path=out/(datetime.now(stamp_zone).strftime('%Y%m%d_%H%M%S')+'_'+args.label+'.json')
        path.write_text(json.dumps(result,indent=2,ensure_ascii=False,default=float),encoding='utf-8')
        print('RESULT',path,flush=True)
        progress.close()
        app.processEvents()
    return 1 if 'error' in result else 0


if __name__=='__main__':
    raise SystemExit(main())
