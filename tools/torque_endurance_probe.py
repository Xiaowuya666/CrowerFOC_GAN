"""Bounded loaded/locked-axis Iq hold, progress/cancel, raw CSV and plots.

No implicit calibration, no free-axis constant-torque mode, no motor constants.
Run --help without hardware. A test requires explicit fixture and test limits.
"""
import argparse
import csv
import json
import math
import struct
import sys
import time
from pathlib import Path

import serial
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'HostUI'))
from PyQt5 import QtCore,QtGui,QtWidgets
from crowerfoc_protocol import *
from crowerfoc_transport import SerialWorker,find_port
from torque_endurance_core import Guard,physical_sample,summarize


def options():
    p=argparse.ArgumentParser(description='已标定、带负载/固定轴的短时低电流保持验证；不会自动启动旋转标定。')
    p.add_argument('--load',required=True,choices=('locked','loaded'))
    p.add_argument('--iq',required=True,type=float,help='Iq指令/A，首轮工具范围0<|Iq|<=1')
    p.add_argument('--seconds',required=True,type=float,help='保持时间/s，首轮范围2~60')
    p.add_argument('--max-speed',required=True,type=float,help='实际轴速测试停止阈值/rad/s，范围1~8')
    p.add_argument('--max-travel',required=True,type=float,help='相对试验开始的轴位移范围/rad；固定轴最高0.1')
    a=p.parse_args()
    vals=(a.iq,a.seconds,a.max_speed,a.max_travel)
    if not all(math.isfinite(v) for v in vals): p.error('参数必须有限')
    if not 0<abs(a.iq)<=1 or not 2<=a.seconds<=60: p.error('当前工具仅允许<=1A、<=60s的初轮试验')
    if not 1<=a.max_speed<=8 or not a.max_travel>0: p.error('速度/位移范围无效')
    if a.load=='locked' and a.max_travel>.1: p.error('固定轴允许位移不应超过0.1rad')
    return a


class Capture(SerialWorker):
    def __init__(self,port,callback):
        super().__init__(port); self.callback=callback
    def _dispatch(self,typ,payload):
        super()._dispatch(typ,payload)
        if typ==MSG_TELEMETRY and len(payload)==36:
            x=dict(self.latest)
            x['da'],x['db'],x['dc']=struct.unpack_from('<HHH',bytes(payload),24)
            self.callback(x)


def plot_result(folder,samples,target):
    if not samples: return
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    font_manager.fontManager.addfont(r'C:\Windows\Fonts\msyh.ttc')
    plt.rcParams.update({'font.family':'Microsoft YaHei','axes.unicode_minus':False,'font.size':10})
    fig,axs=plt.subplots(4,1,figsize=(11,11),layout='constrained',sharex=True)
    t=[s['t_s'] for s in samples]
    axs[0].plot(t,[s['requested_iq_A'] for s in samples],'--',label='主机Iq指令',color='#b73131')
    active=[s if s['state']==3 else None for s in samples]
    raw_iq=[]
    for s in active:
        if s is None or 'ia_raw_A' not in s or 'ib_raw_A' not in s:
            raw_iq.append(float('nan')); continue
        a=s['ele']*math.pi/180
        raw_iq.append(-s['ia_raw_A']*math.sin(a)+(s['ia_raw_A']+2*s['ib_raw_A'])/math.sqrt(3)*math.cos(a))
    axs[0].plot(t,raw_iq,color='#bbb',lw=.6,label='同帧原始Iq')
    for key,label,color in (('iq_A','电流环Iq','#1f77b4'),('id_A','电流环Id','#2d9d62')):
        axs[0].plot(t,[s[key] if s else float('nan') for s in active],label=label,color=color)
    axs[0].set(ylabel='d/q电流 / A',title=f'Iq持续保持实测，目标{target:+g}A；未测Nm与温度')
    axs[0].legend(ncol=3)
    for key,color in zip(('ia_A','ib_A','ic_A'),('#1f77b4','#e67e22','#2d9d62')):
        axs[1].plot(t,[s[key] for s in samples],label=key,color=color,lw=.8)
    axs[1].set_ylabel('相电流 / A'); axs[1].legend(ncol=3)
    axs[2].plot(t,[s['vel'] for s in samples],label='轴速',color='#7b4da3')
    axs[2].set_ylabel('速度 / rad/s')
    twin=axs[2].twinx(); twin.plot(t,[s['travel_rad'] for s in samples],color='#2d9d62',label='轴位移')
    twin.set_ylabel('相对轴位移 / rad',color='#2d9d62')
    axs[3].plot(t,[s['vbus_V'] for s in samples],label='母线电压',color='#1f77b4')
    axs[3].set(xlabel='主机接收时间 / s',ylabel='母线 / V')
    for ax in axs: ax.grid(alpha=.2)
    for ext in ('png','svg'): fig.savefig(folder/('hold_curve.'+ext),dpi=180,bbox_inches='tight')
    plt.close(fig)


def main():
    args=options()
    app=QtWidgets.QApplication([]); app.setFont(QtGui.QFont('Microsoft YaHei UI',10))
    progress=QtWidgets.QProgressDialog('检查设备；尚未驱动','立即停止',0,100)
    progress.setWindowTitle('Iq持续保持试验'); progress.setMinimumDuration(0)
    progress.setAutoReset(False); progress.setAutoClose(False); progress.resize(520,140); progress.show()
    report={'settings':vars(args),'temperature_monitored':False,'torque_Nm_measured':False,
            'samples':[],'start_time':time.strftime('%Y-%m-%d %H:%M:%S')}
    params={}; buf=bytearray(); sp=None; r=None
    started=time.monotonic(); next_hb=0.; stage='precheck'; requested=0.; armed=False
    guard=Guard(args.max_speed,args.max_travel,min(1.,max(.5,abs(args.iq)*1.5)))
    reason=None; fault_guard=True

    def on_sample(x):
        nonlocal reason
        row=physical_sample(x,time.monotonic()-started,stage,requested)
        error=guard.check(row,armed) if fault_guard else None
        row['travel_rad']=guard.travel
        report['samples'].append(row)
        if error and reason is None: reason=error

    def send(typ,payload=b''): sp.write(encode_frame(typ,payload))
    def setparam(pid,v): send(MSG_SET,struct.pack('<Hf',pid,v))
    def point(v):
        nonlocal requested
        send(MSG_SETPOINT,struct.pack('<f',v)); requested=v

    def pump(ignore_abort=False):
        nonlocal next_hb
        app.processEvents()
        now=time.monotonic()
        if progress.wasCanceled() and not ignore_abort: raise InterruptedError('用户停止')
        if now>=next_hb:
            send(MSG_HEARTBEAT); next_hb=now+.15
        buf.extend(sp.read(max(1,min(sp.in_waiting,8192)))); r._parse(buf)
        if reason and not ignore_abort: raise RuntimeError(reason)
        if now-started>.7 and (not r.latest or now-r.last_telem>.7) and not ignore_abort:
            raise RuntimeError('遥测丢失超过0.7s')

    def capture(seconds,ignore_abort=False):
        until=time.monotonic()+seconds
        while time.monotonic()<until: pump(ignore_abort)

    def get(pid,ignore_abort=False):
        params.pop(pid,None); send(MSG_GET,struct.pack('<H',pid))
        until=time.monotonic()+.5
        while pid not in params and time.monotonic()<until: pump(ignore_abort)
        if pid not in params: raise TimeoutError(f'参数{pid:04X}回读超时')
        return params[pid]

    def diag(ignore_abort=False):
        return {hex(pid):get(pid,ignore_abort) for pid in (0x0600,0x0601,0x0610,0x0618,0x0619,0x061A,0x020A)}

    try:
        port=find_port()
        if not port: raise RuntimeError('设备未连接；不会自动标定或写入历史换相值')
        r=Capture(port,on_sample); r.param.connect(lambda pid,v:params.update({pid:v}))
        sp=serial.Serial(port,115200,timeout=.01,write_timeout=.2)
        sp.write(stop_packet()); capture(.4)
        if r.latest.get('state')!=1: raise RuntimeError('未确认空闲')
        initial={hex(pid):get(pid) for pid in (0x0100,0x0103,0x0104,0x0200,0x0201,0x0207,0x0303,0x0304,0x0500,0x0620)}
        report['initial_parameters']=initial; report['diagnostics_before']=diag()
        if not r.latest['flags']&4 or initial['0x100']<1 or initial['0x207']<.95:
            raise RuntimeError('换相标定未完成；请先在轴可转动时标定，再固定/加载，保持MCU供电')
        if initial['0x620']!=1: raise RuntimeError('需要已验证的硬件同步采样固件')
        stage='baseline'; capture(1)
        setparam(0x0103,guard.current_limit); setparam(0x0500,0)
        if abs(get(0x0103)-guard.current_limit)>1e-5 or get(0x0500)!=0:
            raise RuntimeError('试验限流或Iq模式未确认')
        send(9,struct.pack('<H',0))
        guard.angle=None; guard.travel=0
        send(MSG_STATE,b'\x03'); point(0)
        until=time.monotonic()+.3
        while time.monotonic()<until and r.latest['state']!=3: pump()
        if r.latest['state']!=3: raise RuntimeError('未确认Iq闭环')
        armed=True; stage='ramp'
        for k in range(1,21): point(args.iq*k/20); capture(.025)
        if abs(get(0x0505)-args.iq)>1e-5: raise RuntimeError('Iq指令回读失败')
        stage='hold'; hold_start=time.monotonic()
        while time.monotonic()-hold_start<args.seconds:
            pump()
            elapsed=time.monotonic()-hold_start
            progress.setLabelText(f'Iq保持 {args.iq:+g}A · {elapsed:.1f}/{args.seconds:g}s\n无自动温度监控；可随时停止')
            progress.setValue(min(99,int(100*elapsed/args.seconds)))
        report['duration_completed']=True
    except Exception as exc:
        report['aborted']=str(exc); print('STOP',exc,flush=True)
    finally:
        if sp is not None:
            armed=False; stage='stop'; requested=0.; fault_guard=False
            try:
                sp.write(stop_packet()); capture(.3,True)
                report['final']=dict(r.latest)
                report['stop_confirmed']=r.latest.get('state')==1
                if report['stop_confirmed']:
                    report['diagnostics_after']=diag(True)
                    if 'initial_parameters' in report:
                        for pid in (0x0103,0x0500):
                            original=report['initial_parameters'][hex(pid)]
                            setparam(pid,original)
                            if abs(get(pid,True)-original)>1e-5: raise RuntimeError(f'原配置{pid:04X}恢复失败')
                        report['configuration_restored']=True
                    capture(.1,True)
                    report['final']=dict(r.latest)
            except Exception as exc:
                report['stop_error']=str(exc)
            sp.close()
        report['summary']=summarize(report['samples'],args.iq)
        report['passed']=bool(report.get('duration_completed') and report.get('stop_confirmed') and
                              not report.get('aborted') and not report.get('stop_error') and
                              report.get('final',{}).get('fault')==0 and
                              report['summary'].get('recorded_hold_s',0)>=args.seconds-.25 and
                              report['summary'].get('mean_tracking_passed'))
        folder=Path(__file__).resolve().parent/'torque_endurance_results'/time.strftime('%Y%m%d_%H%M%S')
        folder.mkdir(parents=True,exist_ok=True)
        (folder/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,default=float),encoding='utf-8')
        if report['samples']:
            fields=sorted(set().union(*(s.keys() for s in report['samples'])))
            with (folder/'telemetry.csv').open('w',newline='',encoding='utf-8-sig') as f:
                writer=csv.DictWriter(f,fieldnames=fields); writer.writeheader(); writer.writerows(report['samples'])
            try: plot_result(folder,report['samples'],args.iq)
            except Exception as exc: print('PLOT_ERROR',exc,flush=True)
        progress.setValue(100); progress.close()
        print('RESULT',folder,'PASSED',report['passed'],flush=True)
        return 0 if report['passed'] else 1


if __name__=='__main__': sys.exit(main())
