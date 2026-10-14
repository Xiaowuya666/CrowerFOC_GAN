"""Bounded no-load hardware acceptance test, visible progress, stop on exit.

Run with HostUI/.venv/Scripts/python.exe tools/drive_full_test.py.
Motor properties are always read/measured; test limits are explicit, not motor defaults.
"""
import json
import argparse
import math
import struct
import sys
import time
from pathlib import Path

import numpy as np
import serial
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'HostUI'))
from PyQt5 import QtCore, QtGui, QtWidgets
from crowerfoc_protocol import *
from crowerfoc_transport import SerialWorker, find_port

parser = argparse.ArgumentParser()
parser.add_argument('--velocity-only', action='store_true')
parser.add_argument('--vel-gain', type=float)
parser.add_argument('--vel-ki', type=float)
args = parser.parse_args()

app = QtWidgets.QApplication([])
app.setFont(QtGui.QFont('Microsoft YaHei UI', 10))
progress = QtWidgets.QProgressDialog('连接设备', '停止测试', 0, 100)
progress.setWindowTitle('CrowerFOC 空载驱动测试')
progress.setMinimumDuration(0)
progress.setAutoClose(False)
progress.setAutoReset(False)
progress.resize(520, 140)
progress.show()


class Axis:
    def __init__(self):
        port = find_port()
        if not port:
            raise RuntimeError('USB 虚拟串口未枚举')
        self.port = serial.Serial(port, 921600, timeout=.008, write_timeout=.3)
        self.worker = SerialWorker(port)
        self.params = {}
        self.worker.param.connect(lambda k, v: self.params.update({k: v}))
        self.buf = bytearray()
        self.samples = []
        self.next_hb = 0
        self.heartbeat = True
        self.started = time.monotonic()

    def send(self, t, p=b''):
        self.port.write(encode_frame(t, p))

    def set(self, pid, v):
        self.send(MSG_SET, struct.pack('<Hf', pid, v))

    def state(self, s):
        self.send(MSG_STATE, bytes([s]))

    def point(self, v):
        self.send(MSG_SETPOINT, struct.pack('<f', v))

    def read(self, guard=True):
        app.processEvents()
        if progress.wasCanceled():
            raise InterruptedError('用户停止测试')
        if self.heartbeat and time.monotonic() >= self.next_hb:
            self.send(MSG_HEARTBEAT)
            self.next_hb = time.monotonic() + .15
        old = self.worker.count
        self.buf.extend(self.port.read(max(1, min(self.port.in_waiting, 8192))))
        self.worker._parse(self.buf)
        if old != self.worker.count:
            x = {k: v.item() if isinstance(v, np.generic) else v
                 for k, v in self.worker.latest.items()}
            x['t'] = time.monotonic() - self.started
            self.samples.append(x)
            if guard and x['state'] != 1:
                if x['fault'] or abs(x['vel']) > 8 or max(abs(x[k]) for k in ('ia', 'ib', 'ic')) > 3500:
                    self.port.write(stop_packet())
                    raise RuntimeError(f'测试保护停止: {x}')
        return self.worker.latest

    def capture(self, seconds, guard=True):
        begin = len(self.samples)
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.read(guard)
        if not self.worker.latest or time.monotonic() - self.worker.last_telem > .7:
            raise RuntimeError('遥测超时')
        return self.samples[begin:]

    def get(self, pid):
        self.params.pop(pid, None)
        self.send(MSG_GET, struct.pack('<H', pid))
        end = time.monotonic() + .5
        while pid not in self.params and time.monotonic() < end:
            self.read()
        if pid not in self.params:
            raise TimeoutError(f'参数 {pid:04X} 回读超时')
        return self.params[pid]

    def stop(self):
        self.port.write(stop_packet())
        self.capture(.2, False)
        assert self.worker.latest['state'] == 1, '未确认空闲'

    def start(self, mode, target, limit=1.0, if_hz=0, vel_limit=6):
        self.stop()
        assert not self.worker.latest['fault'], '故障未清除'
        for pid, value in ((0x0103, limit), (0x0400, vel_limit), (0x0500, mode)):
            self.set(pid, value)
        self.send(9, struct.pack('<H', round(if_hz * 10)))
        self.state(3)
        self.point(target)


def metrics(samples):
    assert len(samples) >= 3, '有效遥测不足'
    vel = np.array([s['vel'] for s in samples])
    dq = np.array([[s['id']*.001, s['iq']*.001] for s in samples])
    return dict(frames=len(samples), velocity_mean=float(vel.mean()), velocity_std=float(vel.std()),
                velocity_peak=float(abs(vel).max()), dq_mean_A=dq.mean(axis=0).tolist(),
                dq_std_A=dq.std(axis=0).tolist(), fault=sorted(set(s['fault'] for s in samples)))


report = {'time': time.strftime('%Y-%m-%d %H:%M:%S'),
          'conditions': '用户确认空载、轴自由转动；运行测试限流<=1.2A，目标速度<=6rad/s，主机保护8rad/s；校准电流另由固件配置（新版固定5A）',
          'cases': [], 'samples': []}
a = None


def case(name, work):
    progress.setLabelText(name)
    progress.setValue(min(95, len(report['cases']) * 3))
    print('TEST', name, flush=True)
    result = {'name': name}
    sample_start = len(a.samples) if a else 0
    try:
        result['data'] = work()
        result['passed'] = True
    except AssertionError as exc:
        result.update(passed=False, error=str(exc))
        active = [x for x in a.samples[sample_start:] if x['state'] == 3]
        if len(active) >= 3:
            result['data'] = metrics(active[-200:])
    finally:
        if a:
            a.stop()
        report['cases'].append(result)
        print(json.dumps(result, ensure_ascii=False, default=float), flush=True)


def idle_check():
    a.stop()
    a.send(MSG_CLEAR)
    ss = a.capture(1)
    x = a.worker.latest
    assert x['fault'] == 0 and x['flags'] & 2 and 16000 < x['vbus'] < 26000
    assert a.get(0x0620) == 1
    return dict(telemetry=x, hardware_trigger=a.get(0x0620), metrics=metrics(ss))


def invalid_inputs():
    before = {pid: a.get(pid) for pid in (0x0103, 0x0500, 0x0505, 0x0303)}
    for value in (float('nan'), float('inf'), -1., 0., 21.):
        a.set(0x0103, value)
    a.set(0x0500, 255)
    a.set(0x0303, -1)
    a.point(float('nan'))
    a.state(255)
    corrupt = bytearray(encode_frame(MSG_STATE, b'\x03'))
    corrupt[-1] ^= 1
    a.port.write(corrupt)
    a.capture(.2)
    after = {pid: a.get(pid) for pid in before}
    assert after == before and a.worker.latest['state'] == 1 and not a.worker.latest['fault']
    return dict(before=before, after=after)


def mapping_guard():
    pp = a.get(0x0100)
    if pp != 0:
        return {'skipped': '本次启动已有测得的换相配置', 'pole_pairs': pp}
    a.state(3)
    a.capture(.15, False)
    x = dict(a.worker.latest)
    assert x['state'] == 1 and x['fault'] & 64
    a.send(MSG_CLEAR)
    a.capture(.1)
    return x


def calibration(cancel=False):
    a.set(0x0103, 1)
    if a.get(0x0104) != 5:
        a.set(0x0104, 1)  # Legacy firmware only; new profile is read-only.
    a.state(2)
    if cancel:
        a.capture(.8)
        a.stop()
        assert not a.worker.latest['flags'] & 4
        return dict(a.worker.latest)
    end = time.monotonic() + 35
    seen = False
    stage = -1
    while time.monotonic() < end:
        x = a.read()
        if x:
            seen |= x['state'] == 2
            if seen and x['state'] == 1:
                break
        if time.monotonic() >= getattr(calibration, 'next_poll', 0):
            stage = a.get(0x0208)
            progress.setLabelText('完整标定：' + CAL_STAGES.get(int(stage), str(stage)))
            progress.setValue(min(45, 10 + int(stage) * 6))
            calibration.next_poll = time.monotonic() + .4
    assert seen and a.worker.latest['state'] == 1 and a.worker.latest['flags'] & 4
    assert not a.worker.latest['fault']
    data = {hex(pid): a.get(pid) for pid in (0x0100,0x0101,0x0102,0x0200,0x0201,0x0207)}
    data['calibration_current_A'] = a.get(0x0104)
    dialog = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Information, '完整标定结果',
        f"极对数（自动测得）：{data['0x100']:g}\n方向：{data['0x201']:+g}\n"
        f"偏置：{data['0x200']:.5f} rad\n一致性：{data['0x207']:.4f}\n"
        f"R估算：{data['0x101']:.5f} Ω\nL估算：{data['0x102']*1e6:.2f} µH\n"
        '结果仅在 RAM 生效。继续低电流测试。', QtWidgets.QMessageBox.Ok)
    dialog.show()
    a.capture(1)
    dialog.grab().save(str(Path(__file__).with_name('full_test_calibration_result.png')))
    dialog.close()
    return data


def if_test(target, limit, hz):
    a.start(0, target, limit, hz)
    a.capture(.6)
    ss = a.capture(3)
    m = metrics(ss)
    expected = max(-limit, min(limit, target))
    assert abs(m['dq_mean_A'][1] - expected) < .12, 'Iq平均跟踪偏差超过0.12A'
    return dict(command_A=target, limit_A=limit, electrical_hz=hz, metrics=m)


def torque_test(target):
    direction = a.get(0x0201)
    a.start(0, target, .8)
    ss = a.capture(.10 if abs(target) <= .3 else .045)
    a.stop()
    m = metrics(ss)
    assert target * direction * ss[-1]['vel'] > 0, '速度响应方向错误或未转动'
    return dict(command_A=target, final_velocity=ss[-1]['vel'], metrics=m)


def velocity_test(target, limit=6):
    a.start(1, target, 1, vel_limit=limit)
    a.capture(1.5)
    ss = a.capture(2)
    m = metrics(ss)
    expected = max(-limit, min(limit, target)) * a.get(0x0201)
    assert abs(m['velocity_mean'] - expected) < .5, '稳态速度平均偏差超过0.5rad/s'
    assert m['velocity_std'] < 1.5, '速度波动超过1.5rad/s'
    return dict(command_rad_s=target, limit_rad_s=limit, metrics=m)


def active_lock():
    a.set(0x0500, 0)
    before = {pid: a.get(pid) for pid in (0x0100, 0x0303, 0x0500)}
    a.start(0, .3, .8, 1)
    for pid, v in ((0x0100, before[0x0100]+1), (0x0303, .123), (0x0500, 1)):
        a.set(pid, v)
    a.capture(.2)
    after = {pid: a.get(pid) for pid in before}
    assert before == after
    return dict(before=before, after=after)


def stop_rearm():
    a.start(1, 1, .4)
    a.capture(.5)
    a.stop()
    assert a.get(0x0504) == 0 and a.get(0x0505) == 0
    a.state(3)
    ss = a.capture(.4)
    assert a.get(0x0504) == 0
    return dict(zero_target=a.get(0x0504), metrics=metrics(ss))


def watchdog():
    a.start(1, 1, .4)
    a.capture(.5)
    a.send(MSG_HEARTBEAT)
    a.heartbeat = False
    begin = time.monotonic()
    stopped = None
    try:
        while time.monotonic() - begin < 1.3:
            x = a.read(False)
            if x.get('fault', 0) & 512:
                stopped = time.monotonic() - begin
                break
        assert stopped is not None and .65 < stopped < 1.05
        assert a.worker.latest['state'] == 1
        a.state(3)
        a.capture(.1, False)
        assert a.worker.latest['state'] == 1
        a.send(MSG_CLEAR)
        a.capture(.1, False)
        assert not a.worker.latest['fault'] and a.worker.latest['state'] == 1
        assert a.get(0x0504) == 0
        return dict(stopped_after_seconds=stopped, clear_did_not_restart=True)
    finally:
        a.heartbeat = True


try:
    a = Axis()
    case('空闲、母线、编码器、同步采样健康检查', idle_check)
    if args.vel_gain is not None: a.set(0x0301, args.vel_gain)
    if args.vel_ki is not None: a.set(0x0302, args.vel_ki)
    report['velocity_PI'] = {'kp':a.get(0x0301),'ki':a.get(0x0302)}
    if args.velocity_only:
        assert a.get(0x0100) > 0, '速度测试前需完成标定'
        for target in (1,-1,2,-2,4,-4,6,-6):
            case(f'速度控制 {target:+g}rad/s', lambda v=target: velocity_test(v))
        case('速度目标4rad/s受2rad/s限速约束', lambda: velocity_test(4,2))
        case('停止清零，重新使能不沿用旧指令', stop_rearm)
        case('主机心跳丢失750ms停机、故障锁存、清除不自启', watchdog)
        raise SystemExit(0)
    case('非法数值、CRC错误、非法状态拒绝与协议恢复', invalid_inputs)
    case('未标定换相启动保护', mapping_guard)
    case('取消完整标定，立即回到空闲', lambda: calibration(True))
    case('完整标定、进度和实测结果弹窗', calibration)
    for target, limit, hz in ((.2,.8,1),(.6,.8,1),(1.,1.2,2),(.8,.3,1)):
        case(f'i/f 电流跟踪 {target}A / 限流{limit}A / 电频率{hz}Hz',
             lambda q=target,l=limit,f=hz: if_test(q,l,f))
    for target in (.2, -.2, .3, -.3, .6, -.6):
        case(f'编码器换相短时转矩 {target:+g}A', lambda q=target: torque_test(q))
    for target in (1, -1, 2, -2, 4, -4, 6, -6):
        case(f'速度控制 {target:+g}rad/s', lambda v=target: velocity_test(v))
    case('速度目标4rad/s受2rad/s限速约束', lambda: velocity_test(4, 2))
    case('运行中配置锁定', active_lock)
    case('停止清零，重新使能不沿用旧指令', stop_rearm)
    case('主机心跳丢失750ms停机、故障锁存、清除不自启', watchdog)
    case('最后空闲与采样诊断', lambda: {hex(k):a.get(k) for k in
         (0x0600,0x0601,0x0610,0x0618,0x0619,0x061A,0x020A)})
except Exception as exc:
    report['aborted'] = str(exc)
    print('ABORT', str(exc), flush=True)
finally:
    if a:
        try:
            a.heartbeat = True
            a.port.write(stop_packet())
            # Cancellation cannot prevent the final stop and readback.
            progress.reset()
            a.capture(.25, False)
            report['final'] = dict(a.worker.latest)
            report['samples'] = a.samples
            report['frames'] = a.worker.total
            report['missed'] = a.worker.missed
            if not report.get('aborted'):
                a.set(0x0103, 1)
                a.set(0x0400, 6)
                a.set(0x0500, 0)
                a.capture(.1, False)
        except Exception as exc:
            report['stop_error'] = str(exc)
        a.port.close()
    report['all_passed'] = bool(report['cases']) and not report.get('aborted') and all(c.get('passed') for c in report['cases'])
    out = Path(__file__).resolve().parent / 'drive_full_test_results'
    out.mkdir(exist_ok=True)
    path = out / (time.strftime('%Y%m%d_%H%M%S') + '.json')
    path.write_text(json.dumps(report, indent=2, ensure_ascii=False, default=float), encoding='utf-8')
    progress.setValue(100)
    app.processEvents()
    progress.close()
    print('RESULT', path, 'ALL_PASSED', report['all_passed'], flush=True)
