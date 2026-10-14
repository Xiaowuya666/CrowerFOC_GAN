"""Read-only final device status and parameter snapshot, with heartbeat."""
import json
import struct
import sys
import time
from pathlib import Path
import serial
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'HostUI'))
from PyQt5.QtCore import QCoreApplication
from crowerfoc_protocol import *
from crowerfoc_transport import SerialWorker,find_port
app=QCoreApplication([]); port=find_port(); r=SerialWorker(port); buf=bytearray(); params={}
r.param.connect(lambda k,v:params.update({k:v}))
with serial.Serial(port,115200,timeout=.02,write_timeout=.3) as sp:
    for pid in (0x0100,0x0101,0x0102,0x0103,0x0104,0x0200,0x0201,0x0207,0x020A,
                0x0301,0x0302,0x0400,0x0500,0x0504,0x0505,0x0600,0x0601,0x0610,0x0619,0x061A,0x0620):
        sp.write(encode_frame(MSG_HEARTBEAT))
        sp.write(encode_frame(MSG_GET,struct.pack('<H',pid)))
        end=time.monotonic()+.4
        while pid not in params and time.monotonic()<end:
            buf.extend(sp.read(max(1,sp.in_waiting))); r._parse(buf)
        assert pid in params, f'Missing {pid:04X}'
    end=time.monotonic()+.3
    while time.monotonic()<end:
        buf.extend(sp.read(max(1,sp.in_waiting))); r._parse(buf)
    assert r.latest['state']==1 and r.latest['fault']==0 and params[0x0504]==params[0x0505]==0
result={'telemetry':dict(r.latest),'parameters':{hex(k):v for k,v in params.items()}}
Path(__file__).with_name('final_device_status.json').write_text(json.dumps(result,indent=2,default=float),encoding='utf-8')
print(json.dumps(result,default=float),flush=True)
