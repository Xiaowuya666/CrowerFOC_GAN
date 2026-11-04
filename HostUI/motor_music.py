"""Read user-supplied standard MIDI and upload a monophonic motor score."""
import bisect
import math
import struct
import time
from collections import deque
from pathlib import Path
from PyQt5 import QtCore,QtWidgets
from crowerfoc_protocol import MSG_MUSIC
from system_audio import SystemAudioCapture

MAX_NOTES=1024

def _vlq(data,pos):
    value=0
    for _ in range(4):
        if pos>=len(data):raise ValueError('MIDI 数据不完整')
        b=data[pos];pos+=1;value=(value<<7)|(b&127)
        if not b&128:return value,pos
    raise ValueError('MIDI 可变长度字段过长')

def read_midi(path):
    data=Path(path).read_bytes()
    if len(data)>4*1024*1024 or len(data)<14 or data[:4]!=b'MThd':raise ValueError('不是有效 MIDI 文件')
    size=struct.unpack_from('>I',data,4)[0]
    if size<6 or 8+size>len(data):raise ValueError('MIDI 文件头不完整')
    fmt,ntracks,division=struct.unpack_from('>HHH',data,8)
    if fmt>1 or not division or division&0x8000:raise ValueError('仅支持 PPQN 时基的 MIDI 0/1 文件')
    tracks=[];tempos=[(0,500000)];offset=8+size
    for track in range(ntracks):
        if offset+8>len(data) or data[offset:offset+4]!=b'MTrk':raise ValueError('MIDI 音轨损坏')
        length=struct.unpack_from('>I',data,offset+4)[0];start=offset+8;offset=start+length
        if offset>len(data):raise ValueError('MIDI 音轨被截断')
        buf=data[start:offset];pos=0;tick=0;running=None;notes=[];active={};name=f'音轨 {track+1}'
        while pos<len(buf):
            dt,pos=_vlq(buf,pos);tick+=dt
            status=buf[pos] if pos<len(buf) else 0
            if status&128:pos+=1
            elif running is not None:status=running
            else:raise ValueError('无效 MIDI running status')
            if status==0xFF:
                running=None
                if pos>=len(buf):raise ValueError('MIDI meta 数据不完整')
                kind=buf[pos];pos+=1;n,pos=_vlq(buf,pos);payload=buf[pos:pos+n];pos+=n
                if len(payload)!=n:raise ValueError('MIDI meta 数据被截断')
                if kind==0x51 and n==3:
                    tempo=int.from_bytes(payload,'big')
                    if tempo<=0:raise ValueError('MIDI 速度必须为正')
                    tempos.append((tick,tempo))
                if kind==3:name=payload.decode('utf-8','replace')
                if kind==0x2F:break
            elif status in (0xF0,0xF7):
                running=None;n,pos=_vlq(buf,pos);pos+=n
                if pos>len(buf):raise ValueError('MIDI SysEx 数据被截断')
            elif 0x80<=status<=0xEF:
                running=status;kind=status&0xF0;channel=status&15;n=1 if kind in (0xC0,0xD0) else 2
                payload=buf[pos:pos+n];pos+=n
                if len(payload)!=n or any(b>127 for b in payload):raise ValueError('MIDI 音符数据损坏')
                if kind in (0x80,0x90) and channel!=9:
                    key=(channel,payload[0]);on=kind==0x90 and payload[1]>0
                    if on:
                        if key in active:
                            a=active.pop(key)
                            if tick>a:notes.append((a,tick,key[1]))
                        active[key]=tick
                    elif key in active:
                        a=active.pop(key)
                        if tick>a:notes.append((a,tick,key[1]))
            else:raise ValueError('不支持的 MIDI 系统事件')
        for key,a in active.items():
            if tick>a:notes.append((a,tick,key[1]))
        tracks.append((name,notes))
    # Tempo is shared across tracks in format 1. Last event at a tick wins.
    tempo_map=dict(tempos);ticks=sorted(tempo_map);seconds=[0.0]
    for a,b in zip(ticks,ticks[1:]):seconds.append(seconds[-1]+(b-a)*tempo_map[a]/division/1e6)
    def seconds_at(tick):
        i=bisect.bisect_right(ticks,tick)-1
        return seconds[i]+(tick-ticks[i])*tempo_map[ticks[i]]/division/1e6
    return [(name,[(seconds_at(a),seconds_at(b),pitch) for a,b,pitch in notes]) for name,notes in tracks if notes]

def melody_events(notes,transpose=0):
    if not notes:raise ValueError('该音轨没有音符')
    changes={}
    for start,end,pitch in notes:
        if not (math.isfinite(start) and math.isfinite(end) and 0<=start<end<=600):raise ValueError('曲目时长须在10分钟以内')
        changes.setdefault(round(start*1000),[]).append((pitch+transpose,1))
        changes.setdefault(round(end*1000),[]).append((pitch+transpose,-1))
    active={};result=[];previous=0
    for t,events in sorted(changes.items()):
        duration=t-previous
        pitch=max(active) if active else None
        hz=round(440*2**((pitch-69)/12)) if pitch is not None else 0
        if hz and not 80<=hz<=4000:raise ValueError('旋律超出80–4000Hz，请更换音轨或调整移调')
        if duration:
            if result and result[-1][0]==hz:result[-1]=(hz,result[-1][1]+duration)
            else:result.append((hz,duration))
        for pitch,delta in events:
            active[pitch]=active.get(pitch,0)+delta
            if active[pitch]<=0:active.pop(pitch,None)
        previous=t
    # Polyphonic MIDI is reduced to the highest active pitch above. A brief
    # overlap can therefore create a sub-10 ms *derived* segment even when all
    # source note-on/note-off durations are valid. Fold such fragments into an
    # adjacent event instead of rejecting the whole score; total duration stays
    # unchanged and no short note/rest is sent to firmware.
    result=list(result)
    while len(result)>1:
        short_index=min((i for i,(_,ms) in enumerate(result) if ms<10),
                        key=lambda i:result[i][1],default=None)
        if short_index is None:break
        hz,ms=result[short_index]
        if short_index==0:
            neighbor=1
        elif short_index==len(result)-1:
            neighbor=short_index-1
        elif result[short_index-1][0]==hz:
            neighbor=short_index-1
        elif result[short_index+1][0]==hz:
            neighbor=short_index+1
        else:
            neighbor=short_index-1 if result[short_index-1][1]>=result[short_index+1][1] else short_index+1
        nhz,nms=result[neighbor]
        result[neighbor]=(nhz,nms+ms)
        del result[short_index]
        if neighbor>short_index:neighbor-=1
        if neighbor>0 and result[neighbor-1][0]==result[neighbor][0]:
            result[neighbor-1]=(result[neighbor][0],result[neighbor-1][1]+result[neighbor][1])
            del result[neighbor]
        if neighbor+1<len(result) and result[neighbor][0]==result[neighbor+1][0]:
            result[neighbor]=(result[neighbor][0],result[neighbor][1]+result[neighbor+1][1])
            del result[neighbor+1]
    split=[]
    for hz,ms in result:
        if ms<10:raise ValueError('有短于10ms的音符，需先简化 MIDI 装饰音')
        while ms>60000:
            chunk=60000 if ms>=60010 else ms-10
            split.append((hz,chunk));ms-=chunk
        if ms:split.append((hz,ms))
    if not split or len(split)>MAX_NOTES:raise ValueError('曲目超过1024个音符/休止事件，请截取或简化旋律')
    return split

class MotorMusicWidget(QtWidgets.QWidget):
    def __init__(self,parent=None):
        super().__init__(parent);self.worker=None;self.notes=[];self.tracks=[];self.queue=deque();self.pending=None;self.deadline=0
        layout=QtWidgets.QVBoxLayout(self)
        title=QtWidgets.QLabel('电机音乐 · q轴高频电流注入');title.setStyleSheet('font-size:16pt;font-weight:600');layout.addWidget(title)
        self.file_label=QtWidgets.QLabel('请选择 MIDI，或载入内置曲目《你》。');self.file_label.setWordWrap(True);layout.addWidget(self.file_label)
        row=QtWidgets.QHBoxLayout();self.import_button=QtWidgets.QPushButton('导入 MIDI');self.demo_button=QtWidgets.QPushButton('载入《你》');row.addWidget(self.import_button);row.addWidget(self.demo_button);row.addStretch();layout.addLayout(row)
        form=QtWidgets.QFormLayout();self.track=QtWidgets.QComboBox();self.transpose=QtWidgets.QSpinBox();self.transpose.setRange(-24,24);self.transpose.setSuffix(' 半音')
        self.amplitude=QtWidgets.QDoubleSpinBox();self.amplitude.setRange(.05,5.0);self.amplitude.setSingleStep(.1);self.amplitude.setValue(1.0);self.amplitude.setSuffix(' A')
        form.addRow('旋律音轨（和弦取最高音）',self.track);form.addRow('移调',self.transpose);form.addRow('q轴峰值注入电流',self.amplitude);layout.addLayout(form)
        rotation_row=QtWidgets.QHBoxLayout();self.rotate_during_music=QtWidgets.QCheckBox('播放时缓慢旋转（默认关闭）')
        self.rotation_rpm=QtWidgets.QDoubleSpinBox();self.rotation_rpm.setRange(-10.0,10.0);self.rotation_rpm.setDecimals(1);self.rotation_rpm.setSingleStep(.5);self.rotation_rpm.setValue(5.0);self.rotation_rpm.setSuffix(' rpm · 电机轴/ENC1')
        self.rotation_rpm.setEnabled(False);self.rotate_during_music.toggled.connect(self.rotation_rpm.setEnabled)
        rotation_row.addWidget(self.rotate_during_music);rotation_row.addWidget(self.rotation_rpm);rotation_row.addStretch();layout.addLayout(rotation_row)
        self.play=QtWidgets.QCheckBox('播放电机音乐（取消勾选立即停止）');layout.addWidget(self.play)
        audio_box=QtWidgets.QGroupBox('电脑声音 → 电机（实验模式）');audio_layout=QtWidgets.QHBoxLayout(audio_box)
        self.audio_play=QtWidgets.QCheckBox('将 Windows 默认播放声音通过电机播放')
        self.audio_amplitude=QtWidgets.QDoubleSpinBox();self.audio_amplitude.setRange(.05,5.0);self.audio_amplitude.setSingleStep(.25);self.audio_amplitude.setValue(1.0);self.audio_amplitude.setSuffix(' A 峰值 q 轴电流')
        audio_layout.addWidget(self.audio_play,1);audio_layout.addWidget(QtWidgets.QLabel('注入上限'));audio_layout.addWidget(self.audio_amplitude);layout.addWidget(audio_box)
        self.audio_status=QtWidgets.QLabel('PCM 8 kHz 单声道；受电流环和电机机械响应限制，不是高保真声卡。设备空闲、已标定时可启用。')
        self.audio_status.setWordWrap(True);layout.addWidget(self.audio_status)
        self.audio_capture=None;self.audio_pending=None;self.audio_deadline=0.0;self.audio_starting_until=0.0;self.audio_seq=0;self.latest={}
        self.progress=QtWidgets.QProgressBar();layout.addWidget(self.progress);self.status=QtWidgets.QLabel('连接设备并完成电机标定后可播放。');self.status.setWordWrap(True);layout.addWidget(self.status)
        note=QtWidgets.QLabel('q 轴交变电流产生声音；可选叠加 ENC1 电机轴速度闭环，范围 ±10 rpm、默认 5 rpm，默认关闭。旋转与音频共用电流上限，注入越大，可用于维持旋转的电流余量越小。系统音频峰值默认 1 A、上限 5 A；只在固定可靠的台架上使用。停止、断线或故障均退出播放。');note.setWordWrap(True);layout.addWidget(note);layout.addStretch()
        self.import_button.clicked.connect(self.import_midi);self.demo_button.clicked.connect(self.demo);self.play.toggled.connect(self.toggle);self.audio_play.toggled.connect(self.toggle_audio)
        self.timer=QtCore.QTimer(self);self.timer.timeout.connect(self.tick);self.timer.start(100);self.ready=False;self.play.setEnabled(False);self.audio_play.setEnabled(False)
    def set_worker(self,worker):
        self._stop_audio(send_command=bool(self.worker))
        if self.worker:
            try:self.worker.ack.disconnect(self.ack)
            except (TypeError,RuntimeError):pass
        self.worker=worker;self.queue.clear();self.pending=None;self.audio_pending=None
        with QtCore.QSignalBlocker(self.play):self.play.setChecked(False)
        if worker:worker.ack.connect(self.ack)
    def update_device(self,latest,ready):
        playing=latest.get('state')==6
        audio_running=latest.get('state')==7
        self.ready=bool(ready);self.play.setEnabled(bool(self.worker and (ready or playing or self.pending) and not audio_running and not self.audio_pending))
        self.audio_play.setEnabled(bool(self.worker and (ready or audio_running or self.audio_pending) and not playing and not self.pending))
        for widget in (self.import_button,self.demo_button,self.track,self.transpose,self.amplitude,self.rotate_during_music):widget.setEnabled(not playing and not self.pending)
        self.rotation_rpm.setEnabled(self.rotate_during_music.isChecked() and not playing and not self.pending)
        if not playing and not self.pending and self.play.isChecked() and time.monotonic()>getattr(self,'starting_until',0):
            with QtCore.QSignalBlocker(self.play):self.play.setChecked(False)
            self.status.setText('播放已停止 / 完成。' if not latest.get('fault') else '设备故障，播放已停止。')
        self.latest=latest
        if audio_running:
            self.audio_status.setText(f'正在播放系统声音 · 峰值 q 轴电流 {self.audio_amplitude.value():.2f} A · 取消勾选立即停止')
        elif not self.audio_play.isChecked() and not self.audio_pending:
            self.audio_status.setText('PCM 8 kHz 单声道；受电流环和电机机械响应限制，不是高保真声卡。设备空闲、已标定时可启用。')
    def import_midi(self):
        path,_=QtWidgets.QFileDialog.getOpenFileName(self,'选择旋律 MIDI','','MIDI (*.mid *.midi)')
        if not path:return
        self.load_midi(path)
    def load_midi(self,path):
        try:
            tracks=read_midi(path)
            if not tracks:raise ValueError('没有可用旋律音轨（打击乐通道不播放）')
            self.tracks=tracks;self.track.clear()
            for name,notes in tracks:self.track.addItem(f'{name} · {len(notes)}音符')
            self.file_label.setText(str(path));self.notes=[]
        except (ValueError,OSError,struct.error) as exc:QtWidgets.QMessageBox.warning(self,'MIDI 导入失败',str(exc))
    def demo(self):
        self.load_midi(Path(__file__).resolve().parent/'music'/'你_从第15小节.mid')
    def toggle(self,on):
        if not on:
            self.queue.clear();self.pending=None
            if self.worker:self.worker.send_msg(MSG_MUSIC,b'\x03')
            return
        try:
            if not self.ready or not self.worker:raise ValueError('设备须空闲、无故障、已标定且支持播放协议')
            notes=melody_events(self.tracks[self.track.currentIndex()][1],self.transpose.value()) if self.tracks else self.notes
            if not notes:raise ValueError('先导入 MIDI 或载入测试音')
            self.queue=deque([b'\x00'+struct.pack('<H',len(notes))])
            for offset in range(0,len(notes),14):self.queue.append(b'\x01'+struct.pack('<H',offset)+b''.join(struct.pack('<HH',*n) for n in notes[offset:offset+14]))
            rotation_rpm=self.rotation_rpm.value() if self.rotate_during_music.isChecked() else 0.0
            self.queue.append(b'\x02'+struct.pack('<ff',self.amplitude.value(),rotation_rpm));self.progress.setRange(0,len(self.queue));self.progress.setValue(0);self.next_packet()
        except ValueError as exc:
            with QtCore.QSignalBlocker(self.play):self.play.setChecked(False)
            self.status.setText(str(exc))
    def next_packet(self):
        self.pending=self.queue.popleft();self.deadline=time.monotonic()+2;self.worker.send_msg(MSG_MUSIC,self.pending)
        self.status.setText('正在上传旋律…' if self.pending[0]!=2 else '正在请求播放…')
    def ack(self,command,status,pid):
        if command==0x8B and self.audio_pending:
            action=self.audio_pending;self.audio_pending=None
            if status:
                self.audio_status.setText(f'设备拒绝系统音频操作，状态码 {status}；请确认空闲、已标定且编码器正常。')
                if action=='start':
                    with QtCore.QSignalBlocker(self.audio_play):self.audio_play.setChecked(False)
                return
            if action=='start':
                self.audio_starting_until=time.monotonic()+1.0
                self.worker.set_audio_active(True)
                self.audio_capture=SystemAudioCapture(self)
                self.audio_capture.pcm_block.connect(self.on_audio_block)
                self.audio_capture.failed.connect(self.on_audio_failed)
                self.audio_capture.start()
                self.audio_status.setText('系统音频已启动，正在捕获 Windows 默认播放设备…')
            else:
                self.audio_status.setText('系统音频已停止，q 轴指令归零。')
            return
        if command!=MSG_MUSIC or self.pending is None:return
        if status:
            self.status.setText(f'设备拒绝播放操作 {self.pending[0]}，状态码 {status}')
            self.queue.clear();self.pending=None
            with QtCore.QSignalBlocker(self.play):self.play.setChecked(False)
            return
        self.progress.setValue(self.progress.value()+1);self.pending=None
        if self.queue:self.next_packet()
        else:
            self.starting_until=time.monotonic()+.7
            self.status.setText('设备已开始播放；取消勾选可立即停止。')
    def tick(self):
        if self.audio_pending and time.monotonic()>self.audio_deadline:
            action=self.audio_pending;self.audio_pending=None
            self.audio_status.setText('系统音频控制应答超时，已停止音频流。')
            self._stop_audio(send_command=True)
            if action=='start':
                with QtCore.QSignalBlocker(self.audio_play):self.audio_play.setChecked(False)
        if self.audio_play.isChecked() and self.latest.get('state')!=7 and time.monotonic()>self.audio_starting_until and not self.audio_pending:
            self._stop_audio(send_command=False)
            with QtCore.QSignalBlocker(self.audio_play):self.audio_play.setChecked(False)
            self.audio_status.setText('设备已退出系统音频模式；检查状态、故障和编码器。')
        if self.pending and time.monotonic()>self.deadline:
            self.toggle(False)
            with QtCore.QSignalBlocker(self.play):self.play.setChecked(False)
            self.status.setText('播放操作应答超时，已请求停止。')

    def toggle_audio(self,on):
        if not on:
            self._stop_audio(send_command=True)
            return
        if self.play.isChecked() or not self.ready or not self.worker:
            self.audio_status.setText('需先停止 MIDI 播放，并确保设备空闲、无故障且已标定。')
            with QtCore.QSignalBlocker(self.audio_play):self.audio_play.setChecked(False)
            return
        self.audio_pending='start';self.audio_deadline=time.monotonic()+2.0
        self.audio_status.setText('正在请求设备进入低电流系统音频模式…')
        # Include the requested sample rate so old firmware fails closed instead
        # of accepting an 8 kHz stream while still consuming it at 2 kHz.
        rotation_rpm=self.rotation_rpm.value() if self.rotate_during_music.isChecked() else 0.0
        self.worker.send_audio_control(b'\x00'+struct.pack('<fHf',self.audio_amplitude.value(),8000,rotation_rpm))

    def _stop_audio(self,send_command):
        capture,self.audio_capture=self.audio_capture,None
        if capture:
            capture.requestInterruption()
            capture.wait(300)
        if self.worker:
            self.worker.set_audio_active(False)
            self.worker.clear_audio_blocks()
            if send_command:self.worker.send_audio_control(b'\x02')
        if self.audio_pending=='start':self.audio_pending=None

    def on_audio_block(self,sequence,pcm):
        if self.worker and self.audio_play.isChecked():
            self.worker.queue_audio_block(sequence,pcm)

    def on_audio_failed(self,message):
        self._stop_audio(send_command=True)
        with QtCore.QSignalBlocker(self.audio_play):self.audio_play.setChecked(False)
        self.audio_status.setText(message+'。请在 HostUI/.venv 中安装 soundcard：python -m pip install soundcard')
