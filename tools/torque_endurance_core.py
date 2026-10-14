"""Offline guards/statistics for bounded Iq endurance tests. No device access."""
import math
import numpy as np


def physical_sample(sample, elapsed, stage, requested_iq):
    row = {k: v.item() if isinstance(v, np.generic) else v for k, v in sample.items()}
    row.update(t_s=elapsed, stage=stage, requested_iq_A=requested_iq)
    for name in ('ia','ib','ic','id','iq','ia_raw','ib_raw','ic_raw'):
        if name in row:
            row[name+'_A'] = row.pop(name)*.001
    row['vbus_V'] = row.pop('vbus')*.001
    return row


class Guard:
    def __init__(self, max_speed, max_travel, current_limit):
        self.max_speed = max_speed
        self.max_travel = max_travel
        self.current_limit = current_limit
        self.angle = None
        self.travel = 0.
        self.fast_since = None

    def check(self, row, driven):
        angle = row['ang']*math.pi/180
        if self.angle is not None:
            self.travel += (angle-self.angle+math.pi)%(2*math.pi)-math.pi
        self.angle = angle
        if row['fault']:
            return f"设备故障字 {row['fault']}"
        if not 16 <= row['vbus_V'] <= 26:
            return f"母线超出本次已验证范围: {row['vbus_V']:.2f}V"
        if not row['flags'] & 2:
            return '编码器数据无效'
        if not driven:
            return None
        if row['state'] != 3 or row['mode'] != 0:
            return '设备未保持Iq闭环状态'
        if abs(self.travel) > self.max_travel:
            return f'轴位移超过试验范围: {self.travel:.4f}rad'
        # Stationary encoder difference noise can show ~0.76rad/s for one frame.
        # Require 50ms persistence for modest speed excursions; >2x is immediate.
        speed = abs(row['vel'])
        if speed > 2*self.max_speed:
            return f'实际轴速超过硬上限: {speed:.3f}rad/s'
        if speed > self.max_speed:
            if self.fast_since is None:
                self.fast_since = row['t_s']
            if row['t_s']-self.fast_since >= .05:
                return f'实际轴速持续超过试验上限: {speed:.3f}rad/s'
        else:
            self.fast_since = None
        peak = max(abs(row[k+'_A']) for k in ('ia','ib','ic'))
        if peak > max(1., 2*self.current_limit):
            return f'处理后相电流超出试验范围: {peak:.3f}A'
        raw = [abs(row[k+'_A']) for k in ('ia_raw','ib_raw','ic_raw') if k+'_A' in row]
        if raw and max(raw) > 6.45:
            return f'原始采样出现>6.45A异常: {max(raw):.3f}A'
        return None


def summarize(samples, requested_iq):
    # Idle Id/Iq are historical and must never enter hold statistics.
    hold = [s for s in samples if s['stage']=='hold' and s['state']==3 and s['mode']==0]
    if len(hold)<100:
        return {'valid':False,'reason':'保持阶段有效遥测不足100帧'}
    stable = hold[max(0, int(len(hold)*.1)):]
    q = np.array([s['iq_A'] for s in stable])
    d = np.array([s['id_A'] for s in stable])
    phases = np.array([[s[k+'_A'] for k in ('ia','ib','ic')] for s in stable])
    n = max(1,len(q)//3)
    tolerance = max(.05,abs(requested_iq)*.1)
    error = float(q.mean()-requested_iq)
    return {'valid':True,'frames':len(hold),
            'recorded_hold_s':hold[-1]['t_s']-hold[0]['t_s'],
            'iq_mean_A':float(q.mean()),'iq_std_A':float(q.std()),
            'iq_mean_error_A':error,'iq_p95_abs_error_A':float(np.percentile(abs(q-requested_iq),95)),
            'id_mean_A':float(d.mean()),'id_std_A':float(d.std()),
            'iq_late_minus_early_A':float(q[-n:].mean()-q[:n].mean()),
            'phase_rms_A':np.sqrt((phases**2).mean(axis=0)).tolist(),
            'phase_peak_A':abs(phases).max(axis=0).tolist(),
            'velocity_peak_rad_s':max(abs(s['vel']) for s in stable),
            'vbus_minmax_V':[min(s['vbus_V'] for s in stable),max(s['vbus_V'] for s in stable)],
            'duty_clip_fraction':sum(any(s[k]<=41 or s[k]>=1759 for k in ('da','db','dc')) for s in stable)/len(stable),
            'mean_tracking_tolerance_A':tolerance,
            'mean_tracking_passed':abs(error)<=tolerance,
            'meaning':'仅评估Iq保持；没有测量Nm、效率或温度，不是持续额定扭矩认证'}
