"""Export empirical motor-loop figures and CSV; no device access or synthetic traces."""
import csv,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt,font_manager
ROOT=Path(__file__).resolve().parent
OUT=ROOT/'test_visualizations/20261009';OUT.mkdir(parents=True,exist_ok=True)
font_manager.fontManager.addfont('C:/Windows/Fonts/msyh.ttc')
plt.rcParams.update({'font.family':'Microsoft YaHei','axes.unicode_minus':False,'figure.dpi':160,'font.size':10})
def load(label):
 p=sorted((ROOT/'current_loop_results').glob('*_'+label+'.json'))[-1]
 return p,json.loads(p.read_text(encoding='utf-8'))
def segment(d,c,steady=False):
 s=[v for v in d['samples'][c['start']:c['end']] if v['state']==3]
 t=[0.]
 for a,b in zip(s,s[1:]):t.append(t[-1]+((int(b['seq'])-int(a['seq']))&65535)/b.get('stream_hz',100))
 if steady:
  keep=np.array(t)>=t[-1]-1.6;s=[v for v,k in zip(s,keep) if k];t=np.array(t)[keep]
 return s,np.array(t)
def values(s,k):return np.array([v.get(k,np.nan) for v in s],dtype=float)
def save(fig,name):
 fig.tight_layout();fig.savefig(OUT/(name+'.png'));fig.savefig(OUT/(name+'.svg'));plt.close(fig)
labels=['velocity_voltage_sweep','saturation_test','torque_1A_short','timer_observer_torque','timer_observer_velocity','pi_gain_comparison','torque_1A_saturation_hold','final_auto_pi_torque','final_auto_pi_velocity','final_timer_debug_watchdog','phase_waveform_1000hz']
records={};stats=[]
for label in labels:
 p,d=load(label);records[label]=d
 fields=sorted(set().union(*(s.keys() for s in d['samples'])))
 with (OUT/(p.stem+'.csv')).open('w',encoding='utf-8-sig',newline='') as f:
  writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader();writer.writerows(d['samples'])
 for c in d['cases']:
  s,t=segment(d,c,True)
  if not s:continue
  stats.append(dict(label=label,source=p.name,case=c['name'],frames=len(s),
   velocity_mean_rad_s=values(s,'vel').mean(),velocity_std_rad_s=values(s,'vel').std(),
   id_mean_A=values(s,'id').mean()/1000,id_std_A=values(s,'id').std()/1000,
   iq_mean_A=values(s,'iq').mean()/1000,iq_std_A=values(s,'iq').std()/1000,
   iq_error_rms_A=np.sqrt(np.mean((values(s,'iq')/1000-values(s,'iq_target_A'))**2)),
   raw_phase_peak_A=max(np.nanmax(np.abs(values(s,k))) for k in ('ia_raw','ib_raw','ic_raw'))/1000,
   saturation_cycles=c['diagnostics']['0x623']))
(OUT/'summary.json').write_text(json.dumps(stats,indent=2),encoding='utf-8')
with (OUT/'summary.csv').open('w',encoding='utf-8-sig',newline='') as f:
 w=csv.DictWriter(f,fieldnames=stats[0].keys());w.writeheader();w.writerows(stats)
# Full one-second 1A commands: actual current, speed and realizable voltage.
d=records['final_auto_pi_torque'];fig,axes=plt.subplots(3,2,figsize=(12,8),sharex='col')
for col,c in enumerate(d['cases']):
 s,t=segment(d,c)
 for key,title in [('iq_target_A','Iq 指令'),('iq','Iq 实测'),('id','Id 实测')]:axes[0,col].plot(t,values(s,key)/(1000 if key in ('iq','id') else 1),label=title,lw=1)
 axes[0,col].set_title(c['name']+' · 空载');axes[0,col].set_ylabel('电流 / A');axes[0,col].legend(loc='upper right')
 axes[1,col].plot(t,values(s,'vel'));axes[1,col].set_ylabel('机械速度 / rad/s')
 axes[2,col].plot(t,np.hypot(values(s,'vd_V'),values(s,'vq_V')),label='输出电压矢量')
 axes[2,col].plot(t,values(s,'voltage_limit_V'),'--',label='PWM 可实现上限');axes[2,col].set_ylabel('电压 / V');axes[2,col].set_xlabel('设备序号换算时间 / s');axes[2,col].legend()
 for row in range(3):axes[row,col].grid(alpha=.25)
fig.suptitle('实机正反 1A 指令各保持 1 秒：电压饱和后实际 Iq 下降，未过流',fontsize=13);save(fig,'torque_1A_voltage_limit')
# Comparable steady windows; reported velocity is estimator output, not an independent tachometer.
fig,axes=plt.subplots(1,3,figsize=(13,4))
series=[('早期保护版','velocity_voltage_sweep','saturation_test'),('角度观察器 / 时钟修复','timer_observer_velocity','timer_observer_velocity'),('最终自动 PI','final_auto_pi_velocity','final_auto_pi_velocity')]
for name,lo,hi in series:
 arr=[]
 for speed in (8,32,80):
  arr.append(next(v for v in stats if v['label']==(hi if speed==80 else lo) and v['case']==f'velocity_+{speed}'))
 for ax,key in zip(axes,('velocity_std_rad_s','id_std_A','iq_std_A')):ax.plot((8,32,80),[v[key] for v in arr],'o-',label=name)
for ax,title,unit in zip(axes,('速度估计波动','Id 波动','Iq 波动'),('rad/s','A','A')):
 ax.set_title(title);ax.set_xlabel('目标速度 / rad/s');ax.set_ylabel('稳态标准差 / '+unit);ax.grid(alpha=.25)
axes[0].legend(fontsize=8);fig.suptitle('最后 1.6 秒实测对比 · 早期 100Hz，修复后 500Hz；多项修复共同作用');save(fig,'loop_comparison')
# Raw vs conditioned: 16rad/s at 1kHz resolves ~53.5Hz electrical fundamental.
for label,speed,rate,name in (('phase_waveform_1000hz',16,1000,'phase_current_raw_vs_conditioned'),
                              ('final_auto_pi_velocity',80,500,'phase_current_80rad_500hz')):
 d=records[label];c=next(c for c in d['cases'] if c['name']==f'velocity_+{speed}');samp,t=segment(d,c);keep=t>=t[-1]-.1;samp=[v for v,k in zip(samp,keep) if k];t=(t[keep]-t[keep][0])*1000
 fig,axes=plt.subplots(4,1,figsize=(11,8),sharex=True)
 for ax,phase in zip(axes[:3],('a','b','c')):
  ax.plot(t,values(samp,'i'+phase+'_raw')/1000,label='原始',alpha=.7,lw=1);ax.plot(t,values(samp,'i'+phase)/1000,label='调理后',lw=1);ax.set_ylabel('I'+phase+' / A');ax.legend(loc='upper right');ax.grid(alpha=.25)
 axes[3].plot(t,values(samp,'kcl_raw_A'),label='原始 Ia+Ib+Ic');axes[3].axhline(0,color='gray',lw=.7);axes[3].set_ylabel('三相和 / A');axes[3].set_xlabel('设备序号换算时间 / ms');axes[3].legend();axes[3].grid(alpha=.25)
 caution=' · 电气频率约267Hz，500Hz抽样有混叠' if speed==80 else ''
 fig.suptitle(f'最终固件 +{speed}rad/s · 同序号原始与滤波电流（{rate}Hz 抽样）'+caution,fontsize=11);save(fig,name)
# DAP and loss of heartbeat: actual serial traces from final firmware.
d=records['final_timer_debug_watchdog'];s=[v for v in d['samples'] if v['segment'] in ('debug_attach_detach','heartbeat_loss')];t=np.array([v['t'] for v in s]);t-=t[0]
fig,axes=plt.subplots(3,1,figsize=(10,7),sharex=True)
axes[0].plot(t,values(s,'vel'));axes[0].set_ylabel('速度 / rad/s')
axes[1].plot(t,np.where(values(s,'state')==3,values(s,'iq')/1000,np.nan),label='运行 Iq（空闲不计算）');axes[1].plot(t,values(s,'iq_target_A'),label='Iq 指令');axes[1].set_ylabel('电流 / A');axes[1].legend()
axes[2].step(t,values(s,'state'),where='post',label='状态 3=闭环 / 1=空闲');axes[2].step(t,values(s,'fault')/512,where='post',label='COMM 故障');axes[2].set_xlabel('主机记录时间 / s');axes[2].legend()
lost=next(v['t'] for v in s if v['segment']=='heartbeat_loss')-s[0]['t']
for ax in axes:ax.axvline(lost,color='red',ls='--',label='停止心跳');ax.grid(alpha=.25)
fig.suptitle('低速运行中 DAP 连接/断开连续运行；停止心跳后固件自行停机');save(fig,'debug_disconnect_watchdog')
print('OUT',OUT)
for v in stats:
 if v['label']=='final_auto_pi_velocity':print(json.dumps(v))
