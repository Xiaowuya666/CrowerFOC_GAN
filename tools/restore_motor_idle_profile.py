"""Return the bench to idle; optionally clear only the deliberate COMM test fault.
Read R/L and recompute the production calibration PI, never supply motor constants.
"""
import argparse,json,struct,sys,time
from pathlib import Path
from datetime import datetime,timezone,timedelta
import serial
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'HostUI'))
from PyQt5.QtCore import QCoreApplication
from crowerfoc_protocol import *
from crowerfoc_transport import SerialWorker,find_port
args=argparse.ArgumentParser();args.add_argument('--apply-current-autotune',action='store_true');args.add_argument('--clear-comm-only',action='store_true');args=args.parse_args()
app=QCoreApplication([]);r=SerialWorker(find_port());buf=bytearray();params={};r.param.connect(lambda k,v:params.update({k:v}))
result={'writes':[]}
with serial.Serial(r.port,115200,timeout=.01,write_timeout=.3) as sp:
 def pump(seconds):
  end=time.monotonic()+seconds
  while time.monotonic()<end:
   buf.extend(sp.read(max(1,sp.in_waiting)));r._parse(buf)
 def get(pid):
  params.pop(pid,None);sp.write(encode_frame(MSG_GET,struct.pack('<H',pid)))
  end=time.monotonic()+.5
  while pid not in params and time.monotonic()<end:pump(.01)
  if pid not in params:raise RuntimeError(f'Missing {pid:04x}')
  return params[pid]
 def setp(pid,value):
  result['writes'].append({'id':hex(pid),'value':value});sp.write(encode_frame(MSG_SET,struct.pack('<Hf',pid,value)))
  actual=get(pid)
  if abs(actual-value)>max(1e-5,abs(value)*1e-5):raise RuntimeError(f'Write rejected {pid:04x}')
 sp.write(stop_packet());pump(.15)
 result['before']=dict(r.latest)
 if r.latest['state']!=1:raise RuntimeError('Not idle')
 if args.clear_comm_only and r.latest['fault']==512:
  sp.write(encode_frame(MSG_CLEAR));pump(.15)
 if r.latest['fault']:raise RuntimeError('Unexpected fault, preserved')
 if args.apply_current_autotune:
  resistance,inductance=get(0x0101),get(0x0102)
  if not r.latest['flags']&4 or not .0001<resistance<10 or not 1e-7<inductance<.01:raise RuntimeError('No valid measured calibration')
  setp(0x0303,inductance*1500);setp(0x0304,resistance*1500)
 for pid,value in ((0x0103,1),(0x0402,8),(0x0400,10),(0x0500,0),(0x0630,100)):setp(pid,value)
 result['parameters']={hex(pid):get(pid) for pid in (0x0100,0x0101,0x0102,0x0103,0x0104,0x0200,0x0201,0x0303,0x0304,0x0400,0x0402,0x0500,0x0504,0x0505,0x0630)}
 pump(.1);result['final']=dict(r.latest)
 if r.latest['state']!=1 or r.latest['fault'] or get(0x0504)!=0 or get(0x0505)!=0:raise RuntimeError('Final idle check failed')
out=Path(__file__).resolve().parent/'fault_results';stamp=datetime.now(timezone(timedelta(hours=8))).strftime('%Y%m%d_%H%M%S');path=out/(stamp+'_idle_profile.json');path.write_text(json.dumps(result,indent=2,default=float),encoding='utf-8');print('RESULT',path);print(json.dumps(result,default=float))
