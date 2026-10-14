"""USB acceptance for extended joint firmware; public inputs are rpm/deg/A.
Run --snapshot first; --accept performs bounded 5 A calibration and <=1 A motion.
Progress/results are visible. Every motion path stops in finally.
"""
import argparse,csv,hashlib,json,math,struct,sys,time
from datetime import datetime,timezone,timedelta
from pathlib import Path
import numpy as np
import serial
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'HostUI'))
from PyQt5 import QtWidgets
from crowerfoc_protocol import *
from crowerfoc_transport import SerialWorker,find_port
ROOT=Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser();parser.add_argument('--snapshot',action='store_true');parser.add_argument('--accept',action='store_true')
args=parser.parse_args()
app=QtWidgets.QApplication([])
progress=QtWidgets.QProgressDialog('连接USB','停止测试',0,100)
progress.setWindowTitle('关节功能回归 · 小电流');progress.setMinimumDuration(0);progress.setAutoClose(False);progress.show()
from crowerfoc_client import USBClient
def Axis():return USBClient(progress=progress)
def plain(x):
    if isinstance(x,dict):return {str(k):plain(v) for k,v in x.items()}
    if isinstance(x,(list,tuple)):return [plain(v) for v in x]
    if isinstance(x,(float,np.floating)) and not math.isfinite(x):return None
    if isinstance(x,np.generic):return x.item()
    return x
def report_progress(value,label):progress.setValue(value);progress.setLabelText(label);app.processEvents();print(label,flush=True)
axis=None;result={'units':'rpm,deg,A,V,degC','status':'failed','checks':{}}
stamp=datetime.now(timezone(timedelta(hours=8))).strftime('%Y%m%d_%H%M%S')
folder=ROOT/'tools/axis_feature_results';folder.mkdir(exist_ok=True)
try:
    axis=Axis();axis.capture(.3,False)
    ids=(0x0100,0x0101,0x0102,0x0103,0x0200,0x0201,0x0207,0x0301,0x0302,0x0305,0x0310,0x0311,0x0312,0x0400,0x0402,0x0416,0x0422,
         0x0500,0x0501,0x0800,0x0801,0x0802,0x0804,0x0810,0x0811,0x0820,0x0821,0x0822,0x0823,0x0824,0x0825)
    ids=ids+(0x0840,0x0841,0x0842,0x0843,0x0844,0x0845,0x0846)
    result['before']={f'{p:04X}':axis.get(p) for p in ids};result['initial']=dict(axis.worker.latest)
    if args.accept:
        axis.stop();axis.capture(.15,False)
        if axis.worker.latest['fault']:raise RuntimeError(f'启动有故障：{axis.worker.latest["fault"]}')
        board_temp=axis.get(0x0810)
        if math.isfinite(board_temp):
            if board_temp>65:raise RuntimeError('板载温度超过本次台架试验65℃边界，先确认温度或待冷却')
            result['checks']['board_temperature_gate_C']=board_temp
        else:
            if axis.get(0x0437)>0.5:raise RuntimeError('板载NTC已启用但没有有效读数')
            result['checks']['board_temperature_gate']='unavailable; NTC not populated/disabled, run remains low-current and short-duration'
        if axis.get(0x0823)<2:raise RuntimeError('固件没有扩展能力')
        report_progress(5,'1/8 · 检查参数边界与rpm换算')
        previous=axis.get(0x0402);axis.set(0x0402,235)
        assert axis.command(MSG_SET,struct.pack('<Hf',0x0402,to_wire(0x0402,236)),0x0402)==0
        assert axis.command(MSG_SET,struct.pack('<Hf',0x0402,to_wire(0x0402,254)),0x0402)==1
        axis.set(0x0402,previous)
        assert axis.command(MSG_SET,struct.pack('<Hf',0x0103,41),0x0103)==1
        result['checks']['speed_threshold_235rpm']=True;result['checks']['accept_threshold_236rpm']=True
        result['checks']['reject_threshold_254rpm']=True
        result['checks']['reject_current_41A']=True
        calibration_seconds=axis.get(0x020C)
        report_progress(12,f'2/8 · 完整校准（5A，设备预计{calibration_seconds:.1f}秒）')
        calibration_timeout=max(30.0,calibration_seconds+8.0)
        axis.state(2);started=time.monotonic()
        while time.monotonic()-started<calibration_timeout:
            x=axis.read()
            if x and x['fault']:raise RuntimeError(f'校准故障 {x["fault"]}')
            if x and x['state']==1 and time.monotonic()-started>1:
                if not x['flags']&4:raise RuntimeError('校准未有效完成')
                break
            progress.setValue(min(35,12+int((time.monotonic()-started)/max(calibration_seconds,1)*23)))
        else:raise TimeoutError(f'校准超时（设备报告{calibration_seconds:.1f}s，等待{calibration_timeout:.1f}s）')
        result['calibration']={f'{p:04X}':axis.get(p) for p in (0x0100,0x0101,0x0102,0x0200,0x0201,0x0207,0x0303,0x0304)}
        report_progress(36,'3/8 · 保存两份记录，并重启验证')
        axis.set(0x0103,40);axis.set(0x0402,previous)
        for _ in range(2):
            if axis.command(MSG_SAVE):raise RuntimeError('Flash保存失败')
        sequence=axis.get(0x0821);axis.send(MSG_REBOOT);axis.port.close();axis=None
        end=time.monotonic()+7
        while time.monotonic()<end:
            try:axis=Axis();axis.capture(.4,False);break
            except (serial.SerialException,RuntimeError):
                if axis:axis.port.close();axis=None
                app.processEvents();time.sleep(.1)
        if not axis:raise RuntimeError('重启后USB未恢复')
        assert axis.get(0x0820)==1 and axis.get(0x0821)==sequence
        assert axis.get(0x0103)==40 and axis.worker.latest['flags']&4 and axis.worker.latest['state']==1
        result['checks']['flash_reload']=True;result['checks']['saved_sequence']=sequence
        report_progress(48,'4/8 · 配置事务拒绝无效组合，不污染运行配置')
        old=axis.get(0x0430);assert axis.command(MSG_CONFIG_PATCH,b'\x00')==0
        assert axis.command(MSG_CONFIG_PATCH,b'\x01'+struct.pack('<Hf',0x0430,95))==0
        assert axis.command(MSG_CONFIG_PATCH,b'\x02')==1
        assert axis.get(0x0430)==old;assert axis.command(MSG_CONFIG_PATCH,b'\x03')==0
        result['checks']['atomic_invalid_restore']=True
        enc2_enabled=axis.get(0x0840)>0.5;enc2_ok=axis.get(0x0843)>0.5;gear_ratio=axis.get(0x0416)
        result['checks']['output_encoder_enabled']=enc2_enabled
        result['checks']['output_encoder_healthy']=enc2_ok
        result['checks']['measured_gear_ratio']=gear_ratio
        output_feedback_ready=enc2_enabled and enc2_ok and math.isfinite(gear_ratio)
        axis.set(0x0103,1);axis.set(0x0400,60);axis.set(0x0500,1);axis.set(0x0501,2)
        axis.set(0x0305,30);axis.set(0x0301,.15/RPM_PER_RAD_S);axis.set(0x0302,.2/RPM_PER_RAD_S)
        if output_feedback_ready:
            report_progress(55,'5/8 · ±30rpm速度控制（运行限流1A）')
            for velocity in (30,-30):
                axis.state(3);axis.point(velocity);rows=axis.capture(4)
                tail=[r['vel'] for r in rows if r['state']==3][-100:]
                mean=float(np.mean(tail));assert abs(mean-velocity)<4,(mean,velocity)
                result['checks'][f'velocity_{velocity}_rpm_mean']=mean
                axis.stop();axis.capture(.8,False)
            report_progress(70,'6/8 · 小角度位置保持与梯形轨迹')
            axis.set(0x0500,2);axis.set(0x0501,4);axis.set(0x0310,15);axis.set(0x0311,30);axis.set(0x0312,30)
            axis.state(3);start=axis.worker.latest['pos'];axis.point(start+20);rows=axis.capture(5)
            error=abs(axis.worker.latest['pos']-(start+20));assert error<3,error
            assert axis.get(0x0816)==1
            result['checks']['position_error_deg']=error
            axis.stop();axis.capture(.8,False)
        else:
            report_progress(55,'5/8 · 验证未连接输出编码器时闭环安全拒绝')
            axis.point(30)
            try:
                axis.state(3)
                raise AssertionError('没有输出轴编码器时固件意外接受了速度闭环')
            except RuntimeError as exc:
                if '设备拒绝状态3' not in str(exc):raise
            blocked=axis.capture(.3,False);latest=axis.worker.latest
            assert latest['state']==1 and latest['fault']&(1<<5),latest
            assert max((abs(r['vel']) for r in blocked),default=0)<20
            result['checks']['speed_position_gate_without_output_encoder']=True
            axis.send(MSG_CLEAR);axis.capture(.2,False)
            assert axis.worker.latest['fault']==0
            report_progress(68,'6/8 · ±0.2A受限力矩点动（1A限流）')
            axis.set(0x0500,0);axis.set(0x0501,1)
            torque_summary={}
            for iq in (.2,-.2):
                if iq<0:
                    axis.point(0);axis.capture(.3,False)
                    deadline=time.monotonic()+2
                    while time.monotonic()<deadline and abs(axis.worker.latest['vel'])>2:axis.capture(.1,False)
                axis.state(3);axis.point(iq);rows=axis.capture(.25)
                assert rows and max(abs(r['vel']) for r in rows)<20
                assert max(max(abs(r['ia_A']),abs(r['ib_A']),abs(r['ic_A'])) for r in rows)<1.5
                iq_rows=[r['iq_A'] for r in rows if math.isfinite(r['iq_A'])]
                torque_summary[f'{iq:+.1f}A']={'mean_iq_A':float(np.mean(iq_rows)),'peak_speed_rpm':max(abs(r['vel']) for r in rows)}
                axis.stop();axis.capture(.3,False)
            assert axis.worker.latest['state']==1 and axis.worker.latest['fault']==0
            result['checks']['bounded_torque_smoke']=torque_summary
            result['checks']['position_skipped_without_output_encoder']=True
        report_progress(82,'7/8 · 主机断心跳关断')
        axis.set(0x0500,0);axis.set(0x0501,1);axis.state(3)
        axis.next_hb=time.monotonic()+10;axis.capture(1,False)
        assert axis.worker.latest['state']==1 and axis.worker.latest['fault']&(1<<9)
        result['checks']['usb_watchdog']=True
        axis.send(MSG_CLEAR);axis.capture(.1,False)
        report_progress(92,'8/8 · 恢复40A配置上限，保持空闲')
        axis.set(0x0103,40);axis.set(0x0301,result['before']['0301']);axis.set(0x0302,result['before']['0302'])
        for p in (0x0305,0x0310,0x0311,0x0312):axis.set(p,result['before'][f'{p:04X}'])
        axis.set(0x0400,230);axis.set(0x0402,253)
        axis.set(0x0500,0);axis.set(0x0501,1)
        if axis.command(MSG_SAVE):raise RuntimeError('最终保存失败')
        axis.capture(.2,False)
        result['checks']['isr_max_us']=axis.get(0x061A)/144
        result['checks']['adc_timeout_diagnostic_count']=axis.get(0x0610)
        assert result['checks']['isr_max_us']<25
        result['final']={f'{p:04X}':axis.get(p) for p in ids}
    result['status']='passed';result['latest']=dict(axis.worker.latest)
except Exception as exc:
    result['error']=str(exc);print('FAIL:',exc,flush=True)
finally:
    if axis:
        result['samples']=axis.samples
        try:axis.close()
        except Exception as exc:result['stop_error']=str(exc)
    elf=ROOT/'Software/MotorTest_cmake/build/debug_GCC_STM32C562CET6/MotorTest.elf'
    result['built_elf_sha256']=hashlib.sha256(elf.read_bytes()).hexdigest()
    path=folder/f'{stamp}_features.json';path.write_text(json.dumps(plain(result),ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
    progress.setValue(100);progress.close()
    dialog=QtWidgets.QMessageBox();dialog.setWindowTitle('关节功能测试结果')
    dialog.setText('测试通过' if result['status']=='passed' else '测试未通过：'+result.get('error','未知'))
    dialog.setInformativeText(str(path));dialog.show();app.processEvents()
    print(json.dumps({'path':str(path),'status':result['status'],'checks':result['checks'],'initial':plain(result.get('initial')),'error':result.get('error')},ensure_ascii=False),flush=True)
    app.processEvents()
if result['status']!='passed':sys.exit(1)
