"""Capture unchanged raw telemetry and 40 kHz spike counters. Always stop on exit."""
import argparse
import json
import math
import os
import struct
import sys
import time
from pathlib import Path
import numpy as np
import serial
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "HostUI"))
from PyQt5 import QtCore
from crowerfoc_transport import SerialWorker, find_port
from crowerfoc_protocol import *

parser = argparse.ArgumentParser()
parser.add_argument("label")
parser.add_argument("--seconds", type=float, default=15)
parser.add_argument("--if-test", action="store_true")
args = parser.parse_args()
app = QtCore.QCoreApplication([])
port = find_port()
deadline = time.monotonic() + 5
while not port and time.monotonic() < deadline:
    time.sleep(.2)
    port = find_port()
if not port:
    raise RuntimeError('CrowerFOC USB port unavailable')
r = SerialWorker(port)
params, samples = {}, []
trace_records = {}
r.param.connect(lambda pid, value: params.update({pid: value}))
r.current_trace.connect(lambda index, values: trace_records.update({index: values}))
buf = bytearray()
sp = serial.Serial(r.port, 921600, timeout=.025, write_timeout=.3)
t0 = time.monotonic()
next_heartbeat = t0


def send(t, p=b""):
    sp.write(encode_frame(t, p))


def read():
    global next_heartbeat
    if time.monotonic() >= next_heartbeat:
        send(MSG_HEARTBEAT)
        next_heartbeat = time.monotonic() + .15
    count = r.count
    buf.extend(sp.read(max(1, min(sp.in_waiting, 8192))))
    r._parse(buf)
    if r.count != count:
        sample = {k: v.item() if isinstance(v, np.generic) else v for k, v in r.latest.items()}
        sample["t"] = round(time.monotonic()-t0, 5)
        samples.append(sample)


def get(pid):
    params.pop(pid, None)
    send(MSG_GET, struct.pack("<H", pid))
    end = time.monotonic() + .4
    while pid not in params and time.monotonic() < end:
        read()
    return params.get(pid)


result = {"label": args.label, "mode": "if" if args.if_test else "idle"}
try:
    sp.write(stop_packet())
    send(MSG_CLEAR)
    end = time.monotonic()+.3
    while time.monotonic()<end:
        read()
    if args.if_test:
        if not r.latest or not r.latest['flags'] & 2 or r.latest['vbus']<16000:
            raise RuntimeError("Encoder/bus invalid")
        send(MSG_SET, struct.pack("<Hf", 0x0103, .8))
        send(MSG_SET, struct.pack("<Hf", 0x061D, 1))
        send(MSG_SET, struct.pack("<Hf", 0x0500, 0))
        send(0x09, struct.pack("<H", 10))
        send(MSG_STATE, b"\x03")
        send(MSG_SETPOINT, struct.pack("<f", .6))
    end = time.monotonic()+args.seconds
    last_print = 0
    while time.monotonic()<end:
        read()
        if time.monotonic()-last_print>2:
            print(json.dumps(r.latest, default=float), flush=True)
            last_print=time.monotonic()
        if args.if_test and r.latest and (r.latest['fault'] or abs(r.latest['vel'])>8):
            result['drive_stopped_reason']=dict(r.latest)
            break
    capture_samples = list(samples)
    sp.write(stop_packet())  # diagnostics must never extend the motor test
    for pid in (0x0600,0x0601,0x0602,0x0603,0x0604,*range(0x0610,0x0621)):
        get(pid)
    result['parameters']={hex(pid): value for pid,value in params.items()}
    if params.get(0x061D)==3:
        for index in range(64):
            send(MSG_GET, struct.pack('<H',0x0700+index))
            deadline=time.monotonic()+.4
            while index not in trace_records and time.monotonic()<deadline:
                read()
        trigger=int(params[0x061E]); next_index=int(params[0x061F])
        offsets=[params[pid] for pid in (0x0615,0x0616,0x0617)]
        result['trace']=[]
        for k in range(64):
            index=(next_index+k)&63
            if index not in trace_records:
                continue
            a,b,c,flags,start,end,min_ccr,direction,ia,ib,ic=trace_records[index]
            delta=(index-trigger)&63
            if delta>16: delta-=64
            result['trace'].append({'t_us':delta*25,'raw_adc':[a,b,c],
                'raw_A':[(v-o)*.03223 for v,o in zip((a,b,c),offsets)],
                'conditioned_A':[ia,ib,ic],'flags':flags,
                'tim_start':start,'tim_end':end,'min_ccr':min_ccr,'down':direction})
finally:
    if sp.is_open:
        sp.write(stop_packet())
    sp.close()
samples = capture_samples
result['samples']=samples
array=np.array([[sample[key]*.001 for key in ('ia','ib','ic')] for sample in samples])
if len(array):
    result['summary']={
        'frames':len(samples), 'mean_A':array.mean(axis=0).tolist(),
        'std_A':array.std(axis=0).tolist(), 'peak_abs_A':abs(array).max(axis=0).tolist(),
        'p99_abs_A':np.percentile(abs(array),99,axis=0).tolist(),
        'kcl_rms_A':float(np.sqrt(np.mean(array.sum(axis=1)**2))),
        'kcl_peak_A':float(abs(array.sum(axis=1)).max()),
        'telemetry_gt_3A_frames':int(np.count_nonzero(abs(array).max(axis=1)>3)),
        'fault_frames':sum(bool(sample['fault']) for sample in samples)}
    driven = [s for s in samples if s['state']==3]
    if driven:
        dq=np.array([[s['id']*.001,s['iq']*.001] for s in driven])
        result['summary']['driven_dq_mean_A']=dq.mean(axis=0).tolist()
        result['summary']['driven_dq_std_A']=dq.std(axis=0).tolist()
    paired = [s for s in driven if 'ia_raw' in s and s['t'] > 2]
    if paired:
        raw=np.array([[s[k]*.001 for k in ('ia_raw','ib_raw','ic_raw')] for s in paired])
        filtered=np.array([[s[k]*.001 for k in ('ia','ib','ic')] for s in paired])
        angle=np.array([s['ele']*math.pi/180 for s in paired])
        raw_beta=(raw[:,0]+2*raw[:,1])/math.sqrt(3)
        raw_dq=np.column_stack((raw[:,0]*np.cos(angle)+raw_beta*np.sin(angle),
                               -raw[:,0]*np.sin(angle)+raw_beta*np.cos(angle)))
        filtered_dq=np.array([[s['id']*.001,s['iq']*.001] for s in paired])
        result['summary']['paired']={
            'frames':len(paired), 'raw_dq_mean_A':raw_dq.mean(axis=0).tolist(),
            'raw_dq_std_A':raw_dq.std(axis=0).tolist(),
            'filtered_dq_mean_A':filtered_dq.mean(axis=0).tolist(),
            'filtered_dq_std_A':filtered_dq.std(axis=0).tolist(),
            'raw_kcl_rms_A':float(np.sqrt(np.mean(raw.sum(axis=1)**2))),
            'filtered_kcl_rms_A':float(np.sqrt(np.mean(filtered.sum(axis=1)**2)))}
folder=Path(__file__).resolve().parent/'current_noise_results'
folder.mkdir(exist_ok=True)
path=folder/(time.strftime('%Y%m%d_%H%M%S')+'_'+args.label+'.json')
path.write_text(json.dumps(result,ensure_ascii=False,indent=2,default=float),encoding='utf-8')
print('RESULT',path)
print(json.dumps(result.get('summary',{})))
print('PARAMETERS',result.get('parameters'))
