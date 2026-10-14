"""Serial transport; all port reads and writes belong to this worker thread."""
import time
import struct
import threading
from collections import deque
import numpy as np
import serial
import serial.tools.list_ports
from PyQt5 import QtCore
from crowerfoc_protocol import *

def find_port():
    for p in serial.tools.list_ports.comports():
        if "0483" in (p.hwid or "").upper() or "CROWER" in (p.description or "").upper():
            return p.device
    return None


# ----------------------------------------------------------------- 串口线程
class SerialWorker(QtCore.QThread):
    connected = QtCore.pyqtSignal()
    status = QtCore.pyqtSignal(str)
    logline = QtCore.pyqtSignal(str)
    param = QtCore.pyqtSignal(int, float)
    stats = QtCore.pyqtSignal(float, int, int)
    ack = QtCore.pyqtSignal(int,int,int)
    sample = QtCore.pyqtSignal(object)

    def __init__(self, port):
        super().__init__()
        self.port = port
        self._stop = threading.Event()
        self._tx = deque()
        self._lock = threading.Lock()
        self._data_lock = threading.RLock()
        self._queued_gets = set()
        self.last_telem = 0.0
        self.sp = None
        self.n = MAXW
        self.buf_fields = ["time_s", "ia", "ib", "ic", "id", "iq", "ang", "ele",
                           "vel", "pos", "da", "db", "dc", "vbus", "fault",
                           "state", "mode", "inmode", "flags"]
        self.data = {k: np.zeros(self.n, np.float32) for k in self.buf_fields}
        self.data["time_s"] = np.zeros(self.n, np.float64)
        self.count = 0
        self.total = 0
        self.missed = 0
        self.seq_last = None
        self.fps = 0.0
        self._t0 = time.monotonic()
        self._origin = self._t0
        self._c0 = 0
        self.latest = {}
        self._raw_pending = None
        self._control_pending = None
        self._axis_pending = None
        self.buf_fields += ["ia_raw", "ib_raw", "ic_raw", "sample_flags"]
        self.data.update({k: np.full(self.n, np.nan, np.float32)
                          for k in ("ia_raw", "ib_raw", "ic_raw", "sample_flags")})

    def run(self):
        sp = None
        for k in range(6):
            try:
                sp = serial.Serial(self.port, 115200, timeout=0.02,
                                   write_timeout=0.2, rtscts=False, dsrdtr=False)
                break
            except Exception as e:
                if k % 5 == 0:
                    self.status.emit("正在打开 %s ...（%s）" % (self.port, str(e)[:40]))
                if self._stop.wait(0.25):
                    return
        if sp is None:
            self.status.emit("打开失败：%s" % self.port)
            return
        self.sp = sp
        self.status.emit("已连接 %s" % self.port)
        self.connected.emit()
        buf = bytearray()
        next_tx = 0.0
        next_heartbeat = 0.0
        try:
            while not self._stop.is_set():
                chunk = sp.read(max(1, min(sp.in_waiting, 8192)))
                if chunk:
                    buf += chunk
                self._parse(buf)
                now = time.monotonic()
                if now >= next_heartbeat and self.last_telem and now-self.last_telem < TELEMETRY_TIMEOUT:
                    self._write(encode_frame(MSG_HEARTBEAT))
                    next_heartbeat = now + 0.2
                if now >= next_tx:
                    with self._lock:
                        packet, pid = self._tx.popleft() if self._tx else (None, None)
                        self._queued_gets.discard(pid)
                    if packet is not None:
                        self._write(packet)
                        next_tx = now + 0.012
                if now - self._t0 >= 0.5:
                    self.fps = (self.count - self._c0) / (now - self._t0)
                    self._t0, self._c0 = now, self.count
                    self.stats.emit(self.fps, self.total, self.missed)
        except Exception as exc:
            self.status.emit("通信中断：%s" % str(exc)[:80])
            self.logline.emit("串口通信失败，已尝试发送停止指令：%s" % exc)
        finally:
            try:
                self._write(stop_packet())
                end = time.monotonic() + 0.12
                while time.monotonic() < end:
                    buf += sp.read(max(1, min(sp.in_waiting, 8192)))
                    self._parse(buf)
            except Exception:
                self.logline.emit("停止指令未能送达；连接已中断，设备状态无法确认")
            sp.close()
            self.sp = None

    def _write(self, packet):
        if self.sp.write(packet) != len(packet):
            raise IOError("串口发送不完整")

    def _parse(self, buf):
        while True:
            i = buf.find(b"\xAA\x55")
            if i < 0:
                if len(buf) > 1:
                    del buf[:-1]
                return
            if len(buf) - i < 4:
                del buf[:i]
                return
            ln = buf[i + 3]
            if ln > 64:
                del buf[:i + 1]
                continue
            total = 5 + ln
            if len(buf) - i < total:
                if i:
                    del buf[:i]
                return
            fr = buf[i:i + total]
            if crc8(fr[2:4 + ln]) == fr[4 + ln]:
                self._dispatch(fr[2], fr[4:4 + ln])
                del buf[:i + total]
            else:
                del buf[:i + 1]

    def _dispatch(self, typ, pl):
        if typ == MSG_ACK and len(pl)==4:
            command,status,pid=struct.unpack('<BBH',bytes(pl))
            self.ack.emit(command,status,pid)
        elif typ==MSG_AXIS_DIAG and len(pl)==62:
            seq,*values=struct.unpack('<H15f',bytes(pl))
            if all(np.isfinite(v) for i,v in enumerate(values) if i not in (7,8,14)):
                values[0]*=DEG_PER_RAD;values[1]*=RPM_PER_RAD_S
                self._axis_pending=(seq,dict(zip(AXIS_FIELDS,values)))
        elif typ == MSG_CONTROL_DIAG and len(pl) == 50:
            seq, *values = struct.unpack('<H12f', bytes(pl))
            if all(np.isfinite(v) for v in values):
                for i in (4,5,6):values[i]*=DEG_PER_RAD
                values[7]*=RPM_PER_RAD_S
                self._control_pending = (seq, dict(zip(CONTROL_FIELDS, values)))
        elif typ == MSG_CURRENT_RAW and len(pl) == 22:
            seq, a, b, c, ia, ib, ic, flags = struct.unpack("<HHHHfffH", bytes(pl))
            if all(np.isfinite(v) for v in (ia, ib, ic)) and max(a, b, c) <= 4095:
                self._raw_pending = (seq, ia*1000, ib*1000, ic*1000, flags, a, b, c)
        elif typ == MSG_TELEMETRY and len(pl) >= 36:
            with self._data_lock:
                self._on_telem(pl)
        elif typ == MSG_PARAM and len(pl) >= 6:
            pid = pl[0] | (pl[1] << 8)
            val = struct.unpack("<f", bytes(pl[2:6]))[0]
            self.param.emit(pid, from_wire(pid,val))
        elif typ == MSG_LOG:
            self.logline.emit(pl.decode("ascii", "replace"))

    def _on_telem(self, pl):
        def u16(o):
            return pl[o] | (pl[o + 1] << 8)

        def i16(o):
            v = u16(o)
            return v - 0x10000 if v & 0x8000 else v

        seq = u16(T["seq"])
        if self.seq_last is not None:
            d = (seq - self.seq_last) & 0xFFFF
            if 1 < d < 0x8000:
                self.missed += d - 1
        self.seq_last = seq
        idx = self.count % self.n
        d = self.data
        host_time = time.time()
        host_monotonic = time.monotonic()
        d["time_s"][idx] = host_monotonic - self._origin
        d["ia"][idx] = i16(T["ia"])
        d["ib"][idx] = i16(T["ib"])
        d["ic"][idx] = i16(T["ic"])
        d["id"][idx] = i16(T["id"])
        d["iq"][idx] = i16(T["iq"])
        d["ang"][idx] = u16(T["ang"]) / 100.0
        d["ele"][idx] = u16(T["ele"]) / 100.0
        d["vel"][idx] = i16(T["vel"]) / 100.0*RPM_PER_RAD_S
        d["pos"][idx] = i16(T["pos"]) / 100.0*DEG_PER_RAD
        d["da"][idx] = u16(T["da"])
        d["db"][idx] = u16(T["db"])
        d["dc"][idx] = u16(T["dc"])
        d["vbus"][idx] = u16(T["vbus"])
        d["fault"][idx] = u16(T["fault"])
        d["state"][idx] = pl[T["state"]]
        d["mode"][idx] = pl[T["mode"]]
        d["inmode"][idx] = pl[T["inmode"]]
        d["flags"][idx] = pl[T["flags"]]
        raw = self._raw_pending
        raw_match = raw is not None and raw[0] == seq
        for j, field in enumerate(("ia_raw", "ib_raw", "ic_raw", "sample_flags"), 1):
            d[field][idx] = raw[j] if raw_match else np.nan
        self._raw_pending = None
        self.latest = {"seq": seq, "host_time": host_time, "host_monotonic": host_monotonic,
                       "ang": d["ang"][idx], "ele": d["ele"][idx],
                       "vel": d["vel"][idx], "pos": d["pos"][idx],
                       "ia": d["ia"][idx], "ib": d["ib"][idx], "ic": d["ic"][idx],
                       "id": d["id"][idx], "iq": d["iq"][idx],
                       "vbus": d["vbus"][idx], "state": int(d["state"][idx]),
                       "mode": int(d["mode"][idx]), "inmode": int(d["inmode"][idx]),
                       "flags": int(d["flags"][idx]), "fault": int(d["fault"][idx])}
        if raw_match:
            self.latest.update(ia_raw=raw[1], ib_raw=raw[2], ic_raw=raw[3],
                               sample_flags=raw[4], adc_raw=list(raw[5:8]))
        if self._control_pending is not None and self._control_pending[0] == seq:
            self.latest.update(self._control_pending[1])
        self._control_pending = None
        if self._axis_pending is not None and self._axis_pending[0]==seq:
            axis=self._axis_pending[1]
            for key,name in (('pos','pos_deg'),('vel','vel_rpm'),('ia','ia_A'),('ib','ib_A'),('ic','ic_A'),('id','id_A'),('iq','iq_A')):
                value=axis[name]*(1000 if key in ('ia','ib','ic','id','iq') else 1)
                self.data[key][idx]=value;self.latest[key]=value
            self.latest.update(axis)
        self._axis_pending=None
        self.count += 1
        self.total += 1
        self.last_telem = time.monotonic()
        self.sample.emit(dict(self.latest))

    def stop(self):
        self._stop.set()

    def safe_idle(self):
        with self._lock:
            self._tx.clear()
            self._queued_gets.clear()
            self._tx.append((stop_packet(), None))

    def send_msg(self, typ, payload=b""):
        if self._stop.is_set():
            return
        pid = struct.unpack("<H", payload)[0] if typ == MSG_GET else None
        with self._lock:
            if typ == MSG_SET:
                write_pid = struct.unpack_from("<H", payload)[0]
                # A queued read before SET cannot confirm that SET. Remove it;
                # the caller's readback must be queued after the write.
                self._tx = deque((packet, read_pid) for packet, read_pid in self._tx
                                 if read_pid != write_pid)
                self._queued_gets.discard(write_pid)
            if pid is not None and pid in self._queued_gets:
                return
            self._tx.append((encode_frame(typ, payload), pid))
            if pid is not None:
                self._queued_gets.add(pid)

    def get_param(self, pid):
        self.send_msg(MSG_GET, struct.pack("<H", pid))

    def set_param(self, pid, val):
        self.send_msg(MSG_SET, struct.pack("<Hf", pid, to_wire(pid,float(val))))

    def set_state(self, st):
        self.send_msg(MSG_STATE, bytes([st]))

    def set_setpoint(self, val):
        mode=int(self.latest.get('mode',0))
        pid=(0x0505,0x0504,0x0503)[mode] if mode in (0,1,2) else 0x0505
        self.send_msg(MSG_SETPOINT, struct.pack("<f", to_wire(pid,float(val))))

    def clear(self):
        self.send_msg(MSG_CLEAR)

    def snapshot(self, win):
        with self._data_lock:
            return self._snapshot_locked(win)

    def _snapshot_locked(self, win):
        c = self.count
        if c == 0:
            return {k: np.zeros(0, np.float32) for k in self.buf_fields}
        size = min(win, c, self.n)
        idx = c % self.n
        start = (idx - size) % self.n
        out = {}
        for k, a in self.data.items():
            if start + size <= self.n:
                out[k] = a[start:start + size].copy()
            else:
                j = self.n - start
                out[k] = np.concatenate((a[start:], a[:size - j]))
        return out


