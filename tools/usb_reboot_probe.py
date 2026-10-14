"""Verify idle software reboot, USB recovery and cleared RAM calibration."""
import json
import struct
import sys
import time
from pathlib import Path
import serial
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'HostUI'))
from PyQt5.QtCore import QCoreApplication
from crowerfoc_protocol import *
from crowerfoc_transport import SerialWorker, find_port
app=QCoreApplication([])
port=find_port()
if not port: raise RuntimeError('No motor USB')
with serial.Serial(port,115200,timeout=.02,write_timeout=.3) as sp:
    sp.write(stop_packet())
    time.sleep(.2)
    sp.write(encode_frame(MSG_REBOOT))
started=time.monotonic()
time.sleep(.9)
deadline=started+15
result={}
while time.monotonic()<deadline:
    port=find_port()
    if not port:
        time.sleep(.2)
        continue
    try:
        with serial.Serial(port,115200,timeout=.02,write_timeout=.3) as sp:
            r=SerialWorker(port); params={}; buf=bytearray()
            r.param.connect(lambda pid,v:params.update({pid:v}))
            until=min(deadline,time.monotonic()+2)
            while time.monotonic()<until:
                buf.extend(sp.read(max(1,sp.in_waiting)))
                r._parse(buf)
                if r.count>=10:
                    sp.write(encode_frame(MSG_GET,struct.pack('<H',0x0100)))
                    if 0x0100 in params:
                        result={'port':port,'recovered_s':time.monotonic()-started,
                                'pole_pairs':params[0x0100], 'state':int(r.latest['state']),
                                'fault':int(r.latest['fault']), 'flags':int(r.latest['flags'])}
                        assert result['pole_pairs']==0 and result['state']==1 and result['fault']==0
                        break
            sp.write(stop_packet())
        if result: break
    except serial.SerialException:
        time.sleep(.2)
if not result: result={'failed':'USB did not recover within 15s'}
path=Path(__file__).with_name('usb_reboot_result.json')
path.write_text(json.dumps(result,indent=2),encoding='utf-8')
print(json.dumps(result),flush=True)
if 'failed' in result: raise SystemExit(1)
