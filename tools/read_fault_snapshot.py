"""GET/telemetry only. Does not clear faults, send heartbeat, change state or arm."""
import json
import struct
import sys
import time
from datetime import datetime, timezone, timedelta
from pathlib import Path
import serial
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'HostUI'))
from PyQt5.QtCore import QCoreApplication
from crowerfoc_protocol import MSG_GET, encode_frame
from crowerfoc_transport import SerialWorker, find_port


def main():
    app = QCoreApplication([])
    port = find_port()
    if not port:
        raise RuntimeError('Motor USB CDC not present')
    worker = SerialWorker(port)
    params, missing, samples = {}, [], []
    worker.param.connect(lambda k, v: params.update({k: v}))
    buf = bytearray()
    last_seq = None
    ids = (0x0100, 0x0101, 0x0102, 0x0103, 0x0104, 0x0200, 0x0201,
           0x0206, 0x0207, 0x0208, 0x020A, 0x020C, 0x020D, 0x020E, 0x020F,
           0x0301, 0x0302, 0x0303, 0x0304, 0x0400, 0x0500, 0x0504, 0x0505,
           0x0600, 0x0601, 0x0602, 0x0603, 0x0604, 0x0610, 0x0615, 0x0616,
           0x0611, 0x0612, 0x0617, 0x0618, 0x0619, 0x061A, 0x0620, 0x0402, 0x0621, 0x0622, 0x0623, 0x0624,
           0x0625, 0x0626, 0x0627, 0x0630,
           *range(0x0210, 0x0215), 0x0217, 0x0218, 0x0219)
    with serial.Serial(port, 115200, timeout=.02, write_timeout=.3) as sp:
        for pid in ids:
            sp.write(encode_frame(MSG_GET, struct.pack('<H', pid)))
            end = time.monotonic() + .35
            while time.monotonic() < end:
                buf.extend(sp.read(max(1, sp.in_waiting)))
                worker._parse(buf)
                if worker.latest and worker.latest['seq'] != last_seq:
                    samples.append(dict(worker.latest))
                    last_seq = worker.latest['seq']
                if pid in params:
                    break
            if pid not in params:
                missing.append(hex(pid))
        end = time.monotonic() + .3
        while time.monotonic() < end:
            buf.extend(sp.read(max(1, sp.in_waiting)))
            worker._parse(buf)
    timestamp = datetime.now(timezone(timedelta(hours=8)))
    result = {'time': timestamp.isoformat(), 'port': port, 'get_only': True,
              'telemetry': dict(worker.latest), 'samples': samples,
              'parameters': {hex(k): v for k, v in params.items()}, 'missing': missing,
              'note': 'Idle d/q are historical values, not current idle phase current.'}
    out = Path(__file__).resolve().parent / 'fault_results'
    out.mkdir(exist_ok=True)
    path = out / (timestamp.strftime('%Y%m%d_%H%M%S') + '_snapshot.json')
    path.write_text(json.dumps(result, indent=2, ensure_ascii=False, default=float), encoding='utf-8')
    print(json.dumps({'path': str(path), 'telemetry': result['telemetry'],
                      'parameters': result['parameters'], 'missing': missing}, ensure_ascii=False, default=float))


if __name__ == '__main__':
    main()
