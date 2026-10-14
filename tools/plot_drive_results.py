"""Reproduce scientific plots and unit-labelled CSV from recorded hardware JSON.
No device access and no synthetic measurement data.
"""
import csv
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'test_visualizations'/'20261008'
OUT.mkdir(parents=True,exist_ok=True)
font_manager.fontManager.addfont(r'C:\Windows\Fonts\msyh.ttc')
plt.rcParams.update({'font.family':'Microsoft YaHei','axes.unicode_minus':False,
                     'font.size':10,'axes.spines.top':False,'axes.spines.right':False,
                     'axes.grid':True,'grid.alpha':.18,'savefig.dpi':180})
sources={
    'before':ROOT/'current_noise_results/20261008_230008_reconnected_trace_if.json',
    'after':ROOT/'current_noise_results/20261008_233219_hardware_if.json',
    'initial':ROOT/'drive_full_test_results/20261008_233431.json',
    'tuned':ROOT/'drive_full_test_results/20261008_233706.json',
    'gui':ROOT/'gui_motion_result.json',
}
data={k:json.loads(p.read_text(encoding='utf-8')) for k,p in sources.items()}
saved=[]

def save(fig,name,caption):
    fig.savefig(OUT/(name+'.png'),bbox_inches='tight')
    fig.savefig(OUT/(name+'.svg'),bbox_inches='tight')
    plt.close(fig)
    saved.append((name,caption))

def series(samples,key,scale=1):
    return np.array([s[key]*scale for s in samples])

def segments(samples,mode):
    groups=[]; g=[]
    for s in samples:
        if s['state']==3 and s['mode']==mode:
            g.append(s)
        elif g:
            groups.append(g); g=[]
    if g: groups.append(g)
    return groups


# Export source telemetry to physical units, and include sequence/flags for pairing.
for name,d in data.items():
    samples=d.get('samples',[])
    with (OUT/(name+'_telemetry.csv')).open('w',newline='',encoding='utf-8-sig') as f:
        columns=['t_s','sequence','mechanical_deg','electrical_deg','velocity_rad_s',
                 'position_rad','ia_A','ib_A','ic_A','id_A','iq_A','ia_raw_A','ib_raw_A',
                 'ic_raw_A','vbus_V','state','mode','fault','flags','sample_flags']
        w=csv.DictWriter(f,fieldnames=columns); w.writeheader()
        for i,s in enumerate(samples):
            row=dict(t_s=s.get('t',i*.01),sequence=s['seq'],mechanical_deg=s['ang'],
                     electrical_deg=s['ele'],velocity_rad_s=s['vel'],position_rad=s['pos'],
                     vbus_V=s['vbus']*.001,state=s['state'],mode=s['mode'],fault=s['fault'],
                     flags=s['flags'],sample_flags=s.get('sample_flags',''))
            for key in ('ia','ib','ic','id','iq','ia_raw','ib_raw','ic_raw'):
                row[key+'_A']=s[key]*.001 if key in s else ''
            w.writerow(row)

# 1: 40kHz diagnostic capture of a real switching-correlated outlier.
trace=[s for s in data['before']['trace'] if -150<=s['t_us']<=150]
fig,ax=plt.subplots(2,1,figsize=(10,7),layout='constrained')
t=np.array([s['t_us'] for s in trace])
for phase,color in enumerate(('#1f77b4','#e67e22','#2d9d62')):
    ax[0].plot(t,[s['raw_A'][phase] for s in trace],'.-',color=color,label=f'原始 {"ABC"[phase]}相')
ax[0].plot(t,[s['conditioned_A'][0] for s in trace],'s--',color='#7b4da3',label='中值+EMA后的 A相')
ax[0].axvline(0,color='#b73131',alpha=.6)
ax[0].set(xlabel='记录周期索引 × 25µs（非独立时间戳）',ylabel='电流 / A',title='实测尖峰：A相原始 -6.72A，处理后 -0.41A')
ax[0].legend(ncol=2)
normal=next(s for s in data['before']['trace'] if s['t_us']==-25)
spike=next(s for s in data['before']['trace'] if s['t_us']==0)
for y,s,color,label in ((1,normal,'#2d9d62','前一周期'),(0,spike,'#b73131','尖峰周期')):
    start=(1799-s['tim_start'])/144
    end=(1799-s['tim_end'])/144
    ax[1].barh(y,end-start,left=start,height=.35,color=color,label=label)
edge=(1799-spike['min_ccr'])/144
ax[1].axvline(edge,color='#e67e22',ls='--',label='该周期最小CCR对应边沿')
ax[1].set(yticks=[0,1],yticklabels=['尖峰周期','前一周期'],xlabel='距TIM1顶点更新的时间 / µs（由计数器推算）',
          title='软件触发的ISR/ADC读取区间跨过比较边沿；不是示波器波形')
ax[1].legend(loc='upper right')
save(fig,'01_spike_capture','40kHz短记录证明软件触发的读取区间在尖峰周期跨越PWM边沿。硬件归因仍需示波器验证。')
with (OUT/'spike_capture_40khz.csv').open('w',newline='',encoding='utf-8-sig') as f:
    w=csv.writer(f); w.writerow(['period_index_us','ia_raw_A','ib_raw_A','ic_raw_A','ia_filtered_A','ib_filtered_A','ic_filtered_A','tim_start','tim_end','min_ccr','direction'])
    for s in data['before']['trace']:
        w.writerow([s['t_us'],*s['raw_A'],*s['conditioned_A'],s['tim_start'],s['tim_end'],s['min_ccr'],s['down']])

# 2: same 0.6A / 1Hz test, retain both raw and conditioned measurements.
fig,axs=plt.subplots(2,2,figsize=(12,8),layout='constrained')
iq_visible=[]
for col,name in enumerate(('before','after')):
    ss=[s for s in data[name]['samples'] if s['state']==3 and s['t']>2 and 'ia_raw' in s]
    angle=series(ss,'ele')*math.pi/180
    ra=series(ss,'ia_raw',.001); rb=series(ss,'ib_raw',.001)
    iq=-ra*np.sin(angle)+(ra+2*rb)/math.sqrt(3)*np.cos(angle)
    tt=series(ss,'t'); t0=tt[0]
    subset=tt-t0<1.5
    iq_visible.extend(iq[subset].tolist())
    axs[0,col].plot(tt[subset]-t0,iq[subset],color='#c5c5c5',lw=.8,label='同帧原始Iq（由A/B及Park角计算）')
    axs[0,col].plot(tt[subset]-t0,series(ss,'iq',.001)[subset],color='#1f77b4',lw=1,label='电流环处理后Iq')
    axs[0,col].axhline(.6,color='#b73131',ls='--',label='0.6A指令')
    axs[0,col].set(title='软件触发 + 滤波' if name=='before' else 'TIM1硬件触发 + 同一滤波',
                   xlabel='时间 / s',ylabel='Iq / A')
    axs[0,col].legend(fontsize=8)
for ax in axs[0]: ax.set_ylim(min(iq_visible)-.08,max(iq_visible)+.08)
params=[data[n]['parameters'] for n in ('before','after')]
counts=[p['0x600'] for p in params]; peaks=[p['0x601']*.03223 for p in params]
for axis,values,title,unit in ((axs[1,0],counts,'10秒测试：40kHz原始 >6.45A 次数','次数'),
                              (axs[1,1],peaks,'40kHz累计原始峰值（包含起停）','电流 / A')):
    axis.bar(['软件触发','硬件触发'],values,color=['#b73131','#2d9d62'],width=.5)
    axis.set(title=title,ylabel=unit,ylim=(0,max(values)*1.3+1))
    for i,v in enumerate(values): axis.text(i,v+.15,f'{v:.2f}' if unit!='次数' else f'{v:g}',ha='center')
fig.suptitle('大尖峰消除与剩余纹波：波形100Hz；计数/峰值来自40kHz原始采样')
save(fig,'02_current_sampling_comparison','同为0.6A/1Hz，>6.45A次数14→0，原始峰值14.02→2.77A。硬件同步仍有纹波和C相偏差；普通曲线不能捕获全部瞬态。')

# 3: physical phase currents, no forced KCL projection.
ss=[s for s in data['after']['samples'] if s['state']==3 and s['t']>2 and 'ia_raw' in s]
fig,axes=plt.subplots(2,1,figsize=(11,7),layout='constrained',sharex=True)
tt=series(ss,'t')
for key,c in zip(('ia','ib','ic'),('#1f77b4','#e67e22','#2d9d62')):
    axes[0].plot(tt,series(ss,key,.001),label=f'{key[1].upper()}相处理后',color=c,lw=1)
raw=sum((series(ss,k,.001) for k in ('ia_raw','ib_raw','ic_raw')))
flt=sum((series(ss,k,.001) for k in ('ia','ib','ic')))
axes[1].plot(tt,raw,color='#aaa',lw=.7,label='原始三相和')
axes[1].plot(tt,flt,color='#7b4da3',lw=1,label='处理后三相和')
for ax in axes: ax.set_ylabel('电流 / A'); ax.legend(ncol=3)
axes[0].set_title('硬件同步采样：三相旋转电流与未掩盖的三相和残差')
axes[1].set_xlabel('从记录开始的时间 / s')
save(fig,'03_phase_currents_and_residual','C相处理后平均约+0.13A，三相和仍有残差；未强制三相和为零，供后续模拟采样检查参考。')

# 4: short bidirectional pulses; angle change is stronger evidence than a single speed sample.
g=segments(data['initial']['samples'],0)
fig,axes=plt.subplots(2,2,figsize=(12,8),layout='constrained')
for ax,group,target in zip(axes.flat,g[6:10],(.3,-.3,.6,-.6)):
    tt=(series(group,'t')-group[0]['t'])*1000
    ax.plot(tt,series(group,'iq',.001),'o-',color='#1f77b4',label='实测Iq')
    ax.axhline(target,color='#b73131',ls='--',label='Iq指令')
    ax.set(title=f'编码器换相 {target:+g}A',xlabel='从首帧开始 / ms',ylabel='Iq / A')
    twin=ax.twinx(); twin.grid(False)
    displacement=np.unwrap(series(group,'ang')*math.pi/180)
    twin.plot(tt,(displacement-displacement[0])*180/math.pi,'s-',color='#2d9d62',label='轴角变化')
    twin.set_ylabel('轴角变化 / °',color='#2d9d62')
    handles,labels=ax.get_legend_handles_labels(); h2,l2=twin.get_legend_handles_labels()
    ax.legend(handles+h2,labels+l2,fontsize=8)
fig.suptitle('闭环双向响应：仅100Hz遥测，0.6A短脉冲约5帧，不能据此测转矩常数')
save(fig,'04_encoder_commutation_pulses','±0.3A和±0.6A短脉冲观察到相应方向轴角变化。±0.2A短脉冲不保证起转，原始失败记录保留。')

# 5: initial versus RAM tuning at every tested velocity.
gs=[segments(data[n]['samples'],1) for n in ('initial','tuned')]
fig,axes=plt.subplots(4,2,figsize=(12,12),layout='constrained')
targets=(1,-1,2,-2,4,-4,6,-6)
for index,(ax,target) in enumerate(zip(axes.flat,targets)):
    for groups,label,color in zip(gs,('原PI：0.05 / 0.1','试调PI：0.15 / 0.2'),('#aaa','#1f77b4')):
        group=groups[index]; tt=series(group,'t')-group[0]['t']
        ax.plot(tt,series(group,'vel'),color=color,lw=.9,label=label)
    ax.axhline(target,color='#b73131',ls='--',label='速度指令')
    ax.set(title=f'{target:+g}rad/s',xlabel='从使能首帧开始 / s',ylabel='机械速度 / rad/s')
    if index==0: ax.legend(fontsize=8)
fig.suptitle('空载速度实测：低速改善仍有齿槽/纹波；试调PI未写死为电机默认值')
save(fig,'05_velocity_before_after','±1rad/s从停滞改善为均值+0.916/-0.864rad/s，低速标准差约0.94~0.99rad/s。平均跟踪通过不代表无低速波动。')

# 6: reference clamp is distinct from actual-speed trip protection.
group=gs[1][8]; tt=series(group,'t')-group[0]['t']
fig,axes=plt.subplots(2,1,figsize=(10,6.5),layout='constrained',sharex=True)
axes[0].plot(tt,series(group,'vel'),label='实际机械速度',color='#1f77b4')
axes[0].axhline(4,color='#aaa',ls=':',label='主机请求4rad/s')
axes[0].axhline(2,color='#b73131',ls='--',label='限幅后目标2rad/s')
axes[0].set(title='速度指令限制2rad/s：稳态均值1.965rad/s，瞬时仍会超调',ylabel='速度 / rad/s'); axes[0].legend()
axes[1].plot(tt,series(group,'iq',.001),label='实测Iq',color='#7b4da3')
axes[1].set(xlabel='从使能首帧开始 / s',ylabel='Iq / A')
save(fig,'06_velocity_reference_limit','速度限制当前仅限制速度模式的目标值，不是实际轴速的过速跳闸。UI已更名并提供提示。')

# 7: watchdog segment plus resulting idle/fault samples.
group=gs[1][-1]
start=group[0]['t']; end=group[-1]['t']
ss=[s for s in data['tuned']['samples'] if start-.1<=s['t']<=end+.45]
tt=series(ss,'t')-start
fig,axes=plt.subplots(3,1,figsize=(10,8),layout='constrained',sharex=True)
for key,color in zip(('ia','ib','ic'),('#1f77b4','#e67e22','#2d9d62')):
    axes[0].plot(tt,series(ss,key,.001),color=color,label=f'{key[1].upper()}相电流')
axes[0].legend(ncol=3)
axes[0].set(ylabel='相电流 / A',title='停止发送心跳：约750ms后设备关闭输出、锁存通信故障')
axes[1].step(tt,series(ss,'state'),where='post',color='#2d9d62',label='状态（3闭环 / 1空闲）')
axes[1].set(ylabel='状态',yticks=[1,3]); axes[1].legend()
axes[2].step(tt,series(ss,'fault'),where='post',color='#b73131',label='故障位（512=通信超时）')
axes[2].set(xlabel='从该次使能首帧开始 / s',ylabel='故障字'); axes[2].legend()
for ax in axes: ax.axvline(.5,color='#666',ls=':',alpha=.7)
axes[0].text(.52,axes[0].get_ylim()[1]*.8,'使能后约0.5s停止心跳',fontsize=9)
save(fig,'07_heartbeat_stop','心跳丢失实测750ms停机；状态回到IDLE，实际相电流回到零附近。故障锁存后手动清除，没有自动重启。')

lines=['# CrowerFOC 实测曲线（2026-10-08）','',
       '条件：空载、轴可自由转动；限流最高1.2A，速度指令±6rad/s，主机测试保护8rad/s。',
       '曲线均来自保存的设备数据。常规遥测100Hz，尖峰短记录按40kHz周期索引；不能替代示波器测量。',
       '每张图均保存PNG及可缩放SVG；各源记录同时导出带单位的CSV。','']
for name,caption in saved:
    lines += [f'## {name}', '',caption,'',f'![{name}]({name}.png)','',f'[SVG]({name}.svg)','']
lines += ['## 数据来源','']+[f'- {k}: `{p.relative_to(ROOT)}`' for k,p in sources.items()]
lines += ['', 'CSV的GUI记录没有单独时间戳，t_s按100Hz名义帧间隔重建；其他CSV使用主机接收时间。',
          '原始JSON保留实际记录值和测试判定，包括失败测试。采样相电流std包含真实正弦变化，噪声对照使用d/q和三相和。']
(OUT/'README.md').write_text('\n'.join(lines),encoding='utf-8')
print('PLOTS',OUT,len(saved))
