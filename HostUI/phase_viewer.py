#!/usr/bin/env python3
"""
CrowerFOC - 3-phase current viewer.

Reads the 17-byte binary frames streamed by the firmware over the USB CDC-ACM
port and visualises the three phase currents, the three PWM duties and the
current space vector (alpha/beta).

Frame: AA 55 seq_lo seq_hi ia_lo ia_hi ib_lo ib_hi ic_lo ic_hi
       da_lo da_hi db_lo db_hi dc_lo dc_hi chk        (chk = XOR of bytes 2..15)

Host -> device command: A5 cmd lo hi chk
       cmd 0x01 set modulation (val = x1000)
       cmd 0x02 set electrical frequency (val = x10)
       cmd 0x03 start streaming
       cmd 0x04 stop streaming
       cmd 0x05 reset counters
"""
import sys
import os
import time
import numpy as np

import serial
import serial.tools.list_ports

from PyQt5 import QtCore, QtWidgets
import pyqtgraph as pg

FRAME_LEN = 17
SYNC = b"\xAA\x55"
PWM_ARR = 1800.0            # TIM1 (ARR+1)
WINDOW = 1200               # default rolling display window (samples)
MAX_WINDOW = 4000           # ring capacity = max selectable window

CMD_MOD, CMD_FREQ, CMD_ON, CMD_OFF, CMD_RESET, CMD_CLRTRIP = 0x01, 0x02, 0x03, 0x04, 0x05, 0x06


def find_port():
    for p in serial.tools.list_ports.comports():
        hwid = (p.hwid or "").upper()
        desc = (p.description or "")
        if "0483" in hwid or "CROWER" in desc.upper():
            return p.device
    return None


class Reader(QtCore.QThread):
    """Reads + parses frames; keeps rolling numpy ring buffers."""
    statsChanged = QtCore.pyqtSignal(float, float, float)  # fps, frames, dropped-est
    statusChanged = QtCore.pyqtSignal(str)

    def __init__(self, port, n=MAX_WINDOW):
        super().__init__()
        self.port = port
        self.n = n
        self._stop = False
        self.sp = None

        self.ia = np.zeros(n, np.float32)   # raw ADC
        self.ib = np.zeros(n, np.float32)
        self.ic = np.zeros(n, np.float32)
        self.da = np.zeros(n, np.float32)   # pulse
        self.db = np.zeros(n, np.float32)
        self.dc = np.zeros(n, np.float32)
        self.count = 0
        self.total = 0
        self.nbytes = 0
        self.seq_last = None
        self.missed = 0
        self.fps = 0.0
        self._t_last = time.time()
        self._c_last = 0

    def run(self):
        try:
            self.sp = serial.Serial(self.port, 921600, timeout=0.05)
        except Exception as e:
            self.statusChanged.emit("OPEN FAILED on %s: %s" % (self.port, e))
            self.statsChanged.emit(0.0, 0, 0)
            return
        self.statusChanged.emit("opened %s" % self.port)
        buf = bytearray()
        while not self._stop:
            chunk = self.sp.read(8192)
            if chunk:
                self.nbytes += len(chunk)
                buf += chunk
            # parse as many complete frames as possible
            while True:
                i = buf.find(SYNC)
                if i < 0:
                    if len(buf) > 1:
                        del buf[:-1]
                    break
                if len(buf) - i < FRAME_LEN:
                    if i > 0:
                        del buf[:i]
                    break
                fr = buf[i:i + FRAME_LEN]
                chk = 0
                for b in fr[2:16]:
                    chk ^= b
                if fr[16] == chk:
                    idx = self.count % self.n
                    self.ia[idx] = fr[4] | (fr[5] << 8)
                    self.ib[idx] = fr[6] | (fr[7] << 8)
                    self.ic[idx] = fr[8] | (fr[9] << 8)
                    self.da[idx] = fr[10] | (fr[11] << 8)
                    self.db[idx] = fr[12] | (fr[13] << 8)
                    self.dc[idx] = fr[14] | (fr[15] << 8)
                    seq = fr[2] | (fr[3] << 8)
                    if self.seq_last is not None:
                        d = (seq - self.seq_last) & 0xFFFF
                        if d > 1:
                            self.missed += d - 1
                    self.seq_last = seq
                    self.count += 1
                    self.total += 1
                del buf[:i + FRAME_LEN]

            now = time.time()
            if now - self._t_last >= 0.5:
                self.fps = (self.count - self._c_last) / (now - self._t_last)
                self._t_last = now
                self._c_last = self.count
                self.statsChanged.emit(self.fps, self.total, self.missed)
        self.sp.close()

    def stop(self):
        self._stop = True
        self.wait(1000)

    def send(self, data):
        try:
            if self.sp is not None:
                self.sp.write(data)
        except Exception:
            pass

    def snapshot(self, win):
        """Return the most recent `win` samples, oldest first."""
        c = self.count
        n = self.n
        if c == 0:
            z = np.zeros(0, np.float32)
            return (z, z, z, z, z, z)
        size = min(win, c, n)
        idx = c % n
        start = (idx - size) % n

        def take(a):
            if start + size <= n:
                return a[start:start + size]
            k = n - start
            return np.concatenate((a[start:], a[:size - k]))

        return (take(self.ia), take(self.ib), take(self.ic),
                take(self.da), take(self.db), take(self.dc))


class Viewer(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("CrowerFOC 3-phase current viewer")
        self.resize(1500, 950)
        self.reader = None
        self.zero = np.zeros(3, np.float64)   # per-phase ADC offset
        self._zeroed = False
        self.window = WINDOW
        self._x = np.arange(MAX_WINDOW, dtype=np.float32)

        # ---------------- scaling (25 mOhm + INA240A2 gain 50 -> 1.25 V/A) ------
        self.shunt_mohm = 25.0
        self.gain = 50.0
        self.vref = 3.3

        # ---------------- top controls ----------------
        tb = QtWidgets.QWidget()
        lay = QtWidgets.QHBoxLayout(tb)
        lay.setContentsMargins(6, 4, 6, 4)

        lay.addWidget(QtWidgets.QLabel("Port"))
        self.cb_port = QtWidgets.QComboBox()
        self.cb_port.setMinimumWidth(110)
        lay.addWidget(self.cb_port)
        self.btn_refresh = QtWidgets.QPushButton("⟳")
        self.btn_refresh.setFixedWidth(28)
        lay.addWidget(self.btn_refresh)
        self.btn_conn = QtWidgets.QPushButton("Connect")
        lay.addWidget(self.btn_conn)

        lay.addSpacing(12)
        lay.addWidget(QtWidgets.QLabel("Mod x1000"))
        self.sl_mod = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.sl_mod.setRange(0, 200)
        self.sl_mod.setValue(0)
        self.sl_mod.setFixedWidth(120)
        self.lb_mod = QtWidgets.QLabel("0")
        lay.addWidget(self.sl_mod); lay.addWidget(self.lb_mod)

        lay.addSpacing(12)
        lay.addWidget(QtWidgets.QLabel("freq x10"))
        self.sl_freq = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.sl_freq.setRange(1, 200)
        self.sl_freq.setValue(20)
        self.sl_freq.setFixedWidth(120)
        self.lb_freq = QtWidgets.QLabel("20")
        lay.addWidget(self.sl_freq); lay.addWidget(self.lb_freq)

        lay.addSpacing(12)
        lay.addWidget(QtWidgets.QLabel("Window"))
        self.sp_window = QtWidgets.QSpinBox()
        self.sp_window.setRange(100, MAX_WINDOW)
        self.sp_window.setSingleStep(100)
        self.sp_window.setValue(self.window)
        self.sp_window.setFixedWidth(90)
        self.sp_window.setToolTip("rolling display window (samples)")
        lay.addWidget(self.sp_window)

        lay.addSpacing(12)
        self.btn_stream = QtWidgets.QPushButton("Stream: ON")
        lay.addWidget(self.btn_stream)
        self.btn_zero = QtWidgets.QPushButton("Zero")
        lay.addWidget(self.btn_zero)
        self.btn_reset = QtWidgets.QPushButton("Reset")
        lay.addWidget(self.btn_reset)
        self.btn_clrtrip = QtWidgets.QPushButton("Clear trip")
        lay.addWidget(self.btn_clrtrip)

        self.lb_stats = QtWidgets.QLabel("disconnected")
        lay.addSpacing(12); lay.addWidget(self.lb_stats)
        lay.addStretch(1)

        # ---------------- plots ----------------
        pg.setConfigOptions(antialias=False, background='k', foreground='w')
        self.pw_cur = pg.PlotWidget(title="Phase currents (mA)")
        self.pw_cur.addLegend()
        self.pw_cur.setLabel('left', 'current', units='mA')
        self.pw_cur.setLabel('bottom', 'sample')
        self.pw_cur.showGrid(x=True, y=True, alpha=0.3)
        self.c_ia = self.pw_cur.plot(pen=pg.mkPen('#ff5555', width=2), name='Ia')
        self.c_ib = self.pw_cur.plot(pen=pg.mkPen('#55ff55', width=2), name='Ib')
        self.c_ic = self.pw_cur.plot(pen=pg.mkPen('#5599ff', width=2), name='Ic')
        for c in (self.c_ia, self.c_ib, self.c_ic):
            c.setClipToView(True)
            c.setDownsampling(auto=True, method='peak')

        self.pw_duty = pg.PlotWidget(title="PWM duty (%)")
        self.pw_duty.addLegend()
        self.pw_duty.setLabel('left', 'duty', units='%')
        self.pw_duty.setLabel('bottom', 'sample')
        self.pw_duty.showGrid(x=True, y=True, alpha=0.3)
        self.d_a = self.pw_duty.plot(pen=pg.mkPen('#ff5555', width=2), name='da')
        self.d_b = self.pw_duty.plot(pen=pg.mkPen('#55ff55', width=2), name='db')
        self.d_c = self.pw_duty.plot(pen=pg.mkPen('#5599ff', width=2), name='dc')
        for c in (self.d_a, self.d_b, self.d_c):
            c.setClipToView(True)
            c.setDownsampling(auto=True, method='peak')

        self.pw_vec = pg.PlotWidget(title="Current space vector (Ia-Ib, alpha/beta)")
        self.pw_vec.setAspectLocked(True)
        self.pw_vec.showGrid(x=True, y=True, alpha=0.3)
        self.pw_vec.setLabel('left', 'beta', units='mA')
        self.pw_vec.setLabel('bottom', 'alpha', units='mA')
        self.v_traj = self.pw_vec.plot(pen=pg.mkPen('#ffcc55', width=1))
        self.v_now = self.pw_vec.plot(pen=None, symbol='o', symbolSize=9,
                                      symbolBrush='#ffffff', symbolPen=None)

        grid = QtWidgets.QGridLayout()
        grid.addWidget(self.pw_cur, 0, 0, 1, 2)
        grid.addWidget(self.pw_duty, 1, 0, 1, 2)
        grid.addWidget(self.pw_vec, 2, 0, 1, 2)

        central = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(central)
        v.setContentsMargins(4, 4, 4, 4)
        v.addWidget(tb)
        v.addLayout(grid, 1)
        self.setCentralWidget(central)

        # ---------------- wiring ----------------
        self.btn_refresh.clicked.connect(self.refresh_ports)
        self.btn_conn.clicked.connect(self.toggle_conn)
        self.sl_mod.valueChanged.connect(lambda val: (self.lb_mod.setText(str(val)),
                                                      self.send_cmd(CMD_MOD, val)))
        self.sl_freq.valueChanged.connect(lambda val: (self.lb_freq.setText(str(val)),
                                                       self.send_cmd(CMD_FREQ, val)))
        self.btn_stream.clicked.connect(self.toggle_stream)
        self.btn_zero.clicked.connect(self.do_zero)
        self.btn_reset.clicked.connect(lambda: self.send_cmd(CMD_RESET, 0))
        self.btn_clrtrip.clicked.connect(lambda: self.send_cmd(CMD_CLRTRIP, 0))
        self.sp_window.valueChanged.connect(self.on_window)

        self.refresh_ports()

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plots)
        self.timer.start(40)

    # ---------------- port handling ----------------
    def refresh_ports(self):
        self.cb_port.clear()
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.cb_port.addItems(ports)
        d = find_port()
        if d and d in ports:
            self.cb_port.setCurrentText(d)

    def toggle_conn(self):
        if self.reader is not None:
            self.reader.stop()
            self.reader = None
            self.btn_conn.setText("Connect")
            self.lb_stats.setText("disconnected")
            return
        port = self.cb_port.currentText().strip()
        if not port:
            return
        self.reader = Reader(port)
        self.reader.statsChanged.connect(self.on_stats)
        self.reader.statusChanged.connect(self.on_status)
        self.reader.start()
        self._zeroed = False
        self.btn_conn.setText("Disconnect")
        self.lb_stats.setText("connecting to %s ..." % port)

    def on_status(self, text):
        self.lb_stats.setText(text)

    def on_stats(self, fps, total, missed):
        nb = self.reader.nbytes if self.reader is not None else 0
        self.lb_stats.setText("rate %.0f Hz   frames %d   missed %d   bytes %d"
                              % (fps, total, missed, nb))

    # ---------------- commands / scaling ----------------
    def send_cmd(self, cmd, val):
        if self.reader is None:
            return
        val &= 0xFFFF
        lo = val & 0xFF
        hi = (val >> 8) & 0xFF
        chk = cmd ^ lo ^ hi
        self.reader.send(bytes([0xA5, cmd, lo, hi, chk]))

    def toggle_stream(self):
        on = self.btn_stream.text().endswith("ON")
        self.send_cmd(CMD_OFF if on else CMD_ON, 0)
        self.btn_stream.setText("Stream: " + ("OFF" if on else "ON"))

    def on_window(self, val):
        self.window = int(val)

    def do_zero(self):
        if self.reader is None:
            return
        ia, ib, ic, _, _, _ = self.reader.snapshot(self.window)
        if ia.size > 0:
            self.zero = np.array([np.mean(ia), np.mean(ib), np.mean(ic)])

    def vals(self, raw_sub, zi):
        # ADC -> mA
        lsb_v = self.vref / 4096.0
        amps_per_v = self.shunt_mohm / 1000.0 * self.gain
        return (raw_sub - zi) * (lsb_v * 1000.0 / amps_per_v)

    # ---------------- plots (flowing scope style) ----------------
    def update_plots(self):
        if self.reader is None:
            return
        r = self.reader
        ia, ib, ic, da, db, dc = r.snapshot(self.window)
        if ia.size == 0:
            return
        if (not self._zeroed) and r.count >= 200:
            self.do_zero()
            self._zeroed = True
        ia = self.vals(ia, self.zero[0])
        ib = self.vals(ib, self.zero[1])
        ic = self.vals(ic, self.zero[2])
        x = self._x[:ia.size]
        self.c_ia.setData(x, ia); self.c_ib.setData(x, ib); self.c_ic.setData(x, ic)
        self.d_a.setData(x, da / PWM_ARR * 100.0)
        self.d_b.setData(x, db / PWM_ARR * 100.0)
        self.d_c.setData(x, dc / PWM_ARR * 100.0)
        # alpha/beta vector: only the most recent slice (keep it light)
        m = ia.size
        s = m - min(m, 400)
        alpha = (2.0 * ia[s:] - ib[s:] - ic[s:]) / 3.0
        beta = (ib[s:] - ic[s:]) / np.sqrt(3.0)
        self.v_traj.setData(alpha, beta)
        if alpha.size:
            self.v_now.setData([alpha[-1]], [beta[-1]])

    def closeEvent(self, ev):
        if self.reader is not None:
            self.reader.stop()
        ev.accept()


def main():
    # ---- high-DPI (4K) support ----
    QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling, True)
    QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_UseHighDpiPixmaps, True)
    app = QtWidgets.QApplication(sys.argv)

    # readable font on high-resolution displays (override with VIEWER_FONT_PT)
    try:
        pt = int(os.environ.get("VIEWER_FONT_PT", "13"))
    except ValueError:
        pt = 13
    f = app.font()
    f.setPointSize(pt)
    app.setFont(f)
    app.setStyleSheet(
        "QLabel{padding:2px;} QPushButton{padding:6px 12px;} "
        "QComboBox{padding:5px;} QSlider{min-height:22px;} QSpinBox{padding:4px;}")

    w = Viewer()
    w.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
