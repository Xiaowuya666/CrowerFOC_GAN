"""CrowerFOC hardware commutation diagnosis; always returns axis to IDLE.
Run with HostUI/.venv/Scripts/python.exe tools/commutation_probe.py idle|calibrate|torque|if
Results are saved under tools/commutation_results/. Torque tests are short, <=0.6 A,
with a 1 A phase-current host guard.
"""
import argparse, json, math, struct, time
from pathlib import Path
import serial
import serial.tools.list_ports

def crc(data):
    c=255
    for b in data:
        c ^= b
        for _ in range(8): c=((c<<1)^7 if c&128 else c<<1)&255
    return c

class Axis:
    def __init__(self):
        ports=[p.device for p in serial.tools.list_ports.comports() if p.vid==0x0483 and p.pid==0x5710]
        if len(ports)!=1: raise RuntimeError(f'Expected one CrowerFOC USB port: {ports}')
        self.port=serial.Serial(ports[0],921600,timeout=.03,write_timeout=.3)
        self.buf=bytearray(); self.commands=[]; self.samples=[]; self.params={}; self.raw_by_seq={}; self.started=time.monotonic(); self.high_current_frames=0; self.guard_seq=None; self.next_heartbeat=0
    def send(self,t,p=b''):
        self.commands.append(dict(t=round(time.monotonic()-self.started,4),type=t,payload=p.hex()))
        d=bytes([t,len(p)])+p
        self.port.write(b'\xaa\x55'+d+bytes([crc(d)]))
    def set(self,i,v): self.send(0x81,struct.pack('<Hf',i,v))
    def state(self,v): self.send(0x83,bytes([v]))
    def point(self,v): self.send(0x84,struct.pack('<f',v))
    def read(self):
        if time.monotonic()>=self.next_heartbeat:
            self.send(0x88); self.next_heartbeat=time.monotonic()+.15
        self.buf.extend(self.port.read(max(1,self.port.in_waiting)))
        while True:
            i=self.buf.find(b'\xaa\x55')
            if i<0:
                if len(self.buf)>1: del self.buf[:-1]
                break
            if i: del self.buf[:i]
            if len(self.buf)<4: break
            n=self.buf[3]+5
            if n>69: del self.buf[0]; continue
            if len(self.buf)<n: break
            f=bytes(self.buf[:n])
            if crc(f[2:-1])!=f[-1]: del self.buf[0]; continue
            del self.buf[:n]
            t=f[2]; p=f[4:-1]
            if t==6 and len(p)==22:
                seq, a, b, c, ia, ib, ic, flags=struct.unpack('<HHHHfffH',p)
                self.raw_by_seq[seq]=dict(raw_adc=[a,b,c],raw_A=[ia,ib,ic],sample_flags=flags)
            elif t==3 and len(p)==6:
                k,v=struct.unpack('<Hf',p); self.params[k]=v
            elif t==1 and len(p)==36:
                u=lambda i: struct.unpack_from('<H',p,i)[0]
                h=lambda i: struct.unpack_from('<h',p,i)[0]
                sample=dict(t=round(time.monotonic()-self.started,4),seq=u(0),raw=u(2),mech=u(4)*.01,
                    elec=u(6)*.01,ia=h(8)*.001,ib=h(10)*.001,ic=h(12)*.001,id=h(14)*.001,iq=h(16)*.001,
                    vel=h(18)*.01,pos=h(20)*.01,vbus=u(22)*.001,fault=u(30),state=p[32],mode=p[33],flags=p[35])
                sample.update(self.raw_by_seq.pop(sample['seq'],{}))
                self.samples.append(sample)
    def capture(self,seconds,driven=False):
        end=time.monotonic()+seconds; last=0
        while time.monotonic()<end:
            self.read()
            if self.samples:
                x=self.samples[-1]
                if self.guard_seq != x['seq']:
                    self.guard_seq=x['seq']
                    phase=x.get('raw_A')
                    if phase:
                        common=sum(phase)/3.0
                        peak=max(abs(v-common) for v in phase)
                    else:
                        peak=max(abs(x[k]) for k in ('ia','ib','ic'))
                    self.high_current_frames = self.high_current_frames+1 if peak>1.0 else 0
                if driven and (x['fault'] or self.high_current_frames>=2 or abs(x['vel'])>8):
                    raise RuntimeError(f'Drive stopped by host guard: {x}')
                if time.monotonic()-last>1:
                    print(json.dumps(x),flush=True); last=time.monotonic()
    def get(self,k):
        self.params.pop(k,None); self.send(0x82,struct.pack('<H',k))
        end=time.monotonic()+.5
        while k not in self.params and time.monotonic()<end: self.read()
        return self.params.get(k)
    def stop(self):
        self.state(1); self.point(0); self.send(9,b'\x00\x00'); self.send(1,b'\x00\x00')

parser=argparse.ArgumentParser(); parser.add_argument('mode',choices=['idle','guard','calibrate','torque','if']); args=parser.parse_args()
a=Axis(); result={'mode':args.mode}; destination=Path(__file__).resolve().parent/'commutation_results'
try:
    a.stop(); a.capture(.8)
    if len(a.samples)<20: raise RuntimeError('No usable telemetry')
    x=a.samples[-1]
    if args.mode!='idle':
        if not 16<x['vbus']<26: raise RuntimeError(f'Unexpected bus voltage {x["vbus"]}')
        if not x['flags']&2: raise RuntimeError('Encoder invalid; inspect SSC word/CRC first')
        a.send(0x86); a.set(0x0103,1.0); a.set(0x0104,1.0); a.set(0x0500,0)
    if args.mode=='guard':
        if a.get(0x0100)!=0: raise RuntimeError('Boot guard test requires unconfigured pole count')
        a.point(0); a.state(3); a.capture(.15)
        x=a.samples[-1]
        if x['state']!=1 or not x['fault']&64: raise RuntimeError(f'Unconfigured closed loop was not rejected: {x}')
        a.send(0x86); result['boot_guard_passed']=True
    elif args.mode=='calibrate':
        a.state(2); a.capture(31,True)
        x=a.samples[-1]
        if x['state']!=1 or not x['flags']&4: raise RuntimeError(f'Calibration not accepted: {x}')
    elif args.mode in ('torque','if'):
        if args.mode=='if': a.send(9,struct.pack('<H',10))
        a.state(3)
        for q in ((.3,.6,-.3,-.6) if args.mode=='torque' else (.6,)):
            a.point(q); a.capture(.04 if args.mode=='torque' and abs(q)>.4 else (.15 if args.mode=='torque' else 3),True)
            a.point(0); a.capture(.25,True)
    result['parameters']={hex(k):a.get(k) for k in [0x0100,0x0101,0x0102,0x0200,0x0201,*range(0x0203,0x020B)]}
except Exception as e:
    result['error']=str(e); print('ERROR:',e,flush=True)
finally:
    a.stop(); a.capture(.3)
    if 'parameters' not in result:
        result['parameters']={hex(k):a.get(k) for k in [0x0100,0x0200,0x0201,*range(0x0203,0x020B)]}
    a.port.close()
    result['commands']=a.commands
    result['samples']=a.samples
    destination.mkdir(exist_ok=True)
    out=destination/(time.strftime('%Y%m%d_%H%M%S')+'_'+args.mode+'.json')
    out.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    print('RESULT',out,flush=True)
    print('PARAMETERS',json.dumps(result['parameters']),flush=True)
if 'error' in result: raise SystemExit(1)
