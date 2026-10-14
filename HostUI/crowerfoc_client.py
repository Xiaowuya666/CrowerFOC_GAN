"""Synchronous USB client. Public values use rpm/deg/A; never starts on connect."""
import math,struct,time
import serial
from PyQt5 import QtCore
from crowerfoc_protocol import *
from crowerfoc_transport import SerialWorker,find_port
class USBClient:
    def __init__(self,progress=None):
        self.progress=progress
        port=find_port()
        if not port:raise RuntimeError('电机USB未枚举')
        self.port=serial.Serial(port,115200,timeout=.01,write_timeout=.3)
        self.worker=SerialWorker(port);self.params={};self.acks=[];self.samples=[];self.buf=bytearray();self.next_hb=0
        self.worker.param.connect(lambda p,v:self.params.update({p:v}))
        self.worker.ack.connect(lambda c,s,p:self.acks.append((c,s,p)))
        dispatch=self.worker._dispatch
        def collect(t,p):
            dispatch(t,p)
            if t==MSG_TELEMETRY:
                x=dict(self.worker.latest);x['t']=time.monotonic();self.samples.append(x)
        self.worker._dispatch=collect
    def send(self,t,p=b''):self.port.write(encode_frame(t,p))
    def read(self,guard=True):
        instance=QtCore.QCoreApplication.instance()
        if instance:instance.processEvents()
        if self.progress and self.progress.wasCanceled():raise InterruptedError('用户取消测试')
        if time.monotonic()>self.next_hb:self.send(MSG_HEARTBEAT);self.next_hb=time.monotonic()+.1
        self.buf.extend(self.port.read(max(1,min(self.port.in_waiting,8192))));self.worker._parse(self.buf)
        x=self.worker.latest
        if guard and x and x['state']!=1:
            lim=8000 if x['state']==2 else 1800
            if x['fault'] or abs(x['vel'])>150 or max(abs(x[k]) for k in ('ia','ib','ic'))>lim:
                self.stop();raise RuntimeError(f'测试边界触发：state={x["state"]}, fault={x["fault"]}, rpm={x["vel"]}')
        return x
    def capture(self,seconds,guard=True):
        first=len(self.samples);end=time.monotonic()+seconds
        while time.monotonic()<end:self.read(guard)
        if not self.worker.latest or time.monotonic()-self.worker.last_telem>.7:raise RuntimeError('遥测超时')
        return self.samples[first:]
    def get(self,pid):
        self.params.pop(pid,None);self.send(MSG_GET,struct.pack('<H',pid));end=time.monotonic()+.7
        while pid not in self.params and time.monotonic()<end:self.read(False)
        if pid not in self.params:raise TimeoutError(f'参数0x{pid:04X}超时')
        return self.params[pid]
    def command(self,t,p=b'',pid=0):
        self.acks.clear();self.send(t,p);end=time.monotonic()+3
        while time.monotonic()<end:
            self.read(False)
            hits=[a for a in self.acks if a[0]==t and a[2]==pid]
            if hits:return hits[-1][1]
        raise TimeoutError(f'命令0x{t:02X}应答超时')
    def set(self,pid,value):
        status=self.command(MSG_SET,struct.pack('<Hf',pid,to_wire(pid,value)),pid)
        if status:raise RuntimeError(f'设备拒绝参数0x{pid:04X}：{status}')
        got=self.get(pid)
        if not math.isclose(got,value,rel_tol=1e-5,abs_tol=.001):raise RuntimeError(f'参数回读不符 {pid:04X} {got} != {value}')
    def state(self,state):
        if self.command(MSG_STATE,bytes([state])):raise RuntimeError(f'设备拒绝状态{state}')
    def point(self,value):
        mode=int(self.get(0x0500));pid=(0x0505,0x0504,0x0503)[mode]
        status=self.command(MSG_SETPOINT,struct.pack('<f',to_wire(pid,value)),pid)
        if status:raise RuntimeError('设定值被拒绝')
    def stop(self):self.port.write(stop_packet())
    def close(self):
        try:self.stop();self.capture(.15,False)
        finally:self.port.close()
