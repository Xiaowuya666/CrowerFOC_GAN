import re
import json
import time
from pathlib import Path
from PyQt5 import QtCore, QtWidgets

ROOT=Path(__file__).resolve().parent.parent
NETLISTS=(('主板',ROOT/'Hardware'/'电调主板.net'),('第二编码器板',ROOT/'Hardware'/'电调第二编码器.net'))

def inventory():
    rows=[]
    for board,path in NETLISTS:
        try:text=path.read_text(encoding='utf-8',errors='replace')
        except OSError:continue
        for ref in re.findall(r'Designator\s*\n([^\n]+)',text):
            ref=ref.strip()
            if ref and ref not in ('DESIGNATOR',):rows.append((board,ref))
    return rows

def failure_modes(ref):
    p=ref.rstrip('0123456789').upper()
    if p in ('R','RT'):return ('开路/裂纹','短路','阻值漂移/焊接不良')
    if p=='C':return ('开路/容量衰减','短路/漏电','ESR升高/焊接不良')
    if p in ('U','LDO'):return ('失电/输出异常','引脚开路','输出短路/热失效')
    if p=='D':return ('开路','短路/反向漏电','焊接不良')
    if p=='L':return ('开路','饱和/电感下降','焊接不良')
    if p in ('CN','FPC','USB','SWD'):return ('断路/接触不良','短路/错接','供电/地异常')
    if p in ('SW',):return ('触点开路','触点短路/卡滞','接触电阻过大')
    if p in ('RGB',):return ('开路/不亮','短路/过流','数据链断裂')
    if p in ('X',):return ('不起振/频偏','引脚开路','负载异常')
    return ('失电/功能失效','引脚开路','引脚短路')

def detection_path(ref,board):
    p=ref.rstrip('0123456789').upper()
    if ref=='U13' or (board=='第二编码器板' and ref=='U1'):return '编码器SSC CRC/STAT、超时和角度连续性；第二编码器走PB6独立CS'
    if p in ('R','C','D','L','SW','RGB','X'):return '软件无法直接判定；按网表节点进行万用表/示波器检查'
    if ref in ('U2','U3','U4','U7','U8','U9'):return '需示波器检查驱动输入/输出与三相开关节点；软件只观察电流、母线及驱动故障'
    if ref in ('U13',):return 'SSC传输、CRC/STAT、超时；启用第二编码器后检查PB6通道'
    if ref in ('U1',):return '心跳、ADC/ISR周期、USB/CAN和看门狗状态'
    if ref in ('U5','U6','U10','U11','U12','LDO1'):return '需测量对应电源/通信/采样节点；软件读数可辅助定位'
    return '需按网表对相关引脚做连续性、电压或波形测量'

def live_checks(latest,params,connected,fresh):
    out=[]
    def add(name,ok,detail):out.append((name,'通过' if ok else '失败/待检查',detail))
    add('USB遥测',connected and fresh,'已连接且遥测未超时' if connected and fresh else '连接后检查USB CDC与帧接收')
    fault=int(latest.get('fault',0) or 0)
    add('驱动故障锁存',fault==0,f'故障位 0x{fault:04X}')
    vbus=float(latest.get('vbus',float('nan')))/1000.0
    add('母线电压',vbus==vbus and 5.0<=vbus<=32.0,f'{vbus:.2f} V' if vbus==vbus else '无有效母线遥测')
    add('电机侧编码器',bool(int(latest.get('flags',0) or 0)&2),'遥测编码器标志正常' if int(latest.get('flags',0) or 0)&2 else '检查TLE5012B U13、PB3/PB4/PB5')
    enabled=bool(params.get(0x0840,0));ok=bool(params.get(0x0843,0))
    add('输出轴第二编码器',ok if enabled else True,('已启用且SSC状态正常' if ok else '已启用但无有效读数') if enabled else '未启用；勾选后才参与反馈')
    ratio=params.get(0x0416,float('nan'))
    if enabled:
        out.append(('传动比估算','通过' if ratio==ratio else '等待运动测量',
                    f'实测电机角/输出角={ratio:.5g}:1' if ratio==ratio else '轻微转动轴后由两编码器实际位移估算'))
    vals=[latest.get(k,float('nan')) for k in ('ia_A','ib_A','ic_A')]
    valid=all(v==v for v in vals)
    add('三相电流遥测',valid,'A/B/C='+', '.join(f'{v:.2f} A' for v in vals) if valid else '电流遥测无效')
    fet=params.get(0x0810,float('nan'))
    if not params.get(0x0437,0):out.append(('板载NTC','跳过','配置未启用；当前未焊接，浮空ADC读数不作为温度证据'))
    else:add('板载NTC',fet==fet and fet<100,f'{fet:.1f} ℃' if fet==fet else '启用但无有效温度读数')
    out.append(('CAN接口','跳过','本机没有CAN适配器；当前只能检查固件离线协议'))
    out.append(('功率器件波形','需仪器','用示波器/差分探头检查U2/U3/U4及三相节点'))
    return out

class DiagnosticDialog(QtWidgets.QDialog):
    def __init__(self,parent,latest,params,connected,fresh):
        super().__init__(parent);self.setWindowTitle('自检与网表故障检查');self.resize(1050,720)
        self.latest=dict(latest);self.params=dict(params);self.connected=connected;self.fresh=fresh
        layout=QtWidgets.QVBoxLayout(self)
        banner=QtWidgets.QLabel('实时自检会读取现有遥测；网表情景表覆盖两份网表中每个器件的典型失效模式。情景是软件建模与仪器检查清单，不会对真实硬件施加短路/开路。')
        banner.setWordWrap(True);layout.addWidget(banner)
        tabs=QtWidgets.QTabWidget();layout.addWidget(tabs)
        live=QtWidgets.QWidget();lv=QtWidgets.QVBoxLayout(live)
        self.live_table=QtWidgets.QTableWidget(0,3);self.live_table.setHorizontalHeaderLabels(['检查项','结果','说明']);self.live_table.horizontalHeader().setStretchLastSection(True)
        for row in live_checks(self.latest,self.params,self.connected,self.fresh):
            n=self.live_table.rowCount();self.live_table.insertRow(n)
            for c,val in enumerate(row):self.live_table.setItem(n,c,QtWidgets.QTableWidgetItem(str(val)))
        lv.addWidget(self.live_table);tabs.addTab(live,'实时自检')
        scenarios=QtWidgets.QWidget();sv=QtWidgets.QVBoxLayout(scenarios)
        self.scenario_table=QtWidgets.QTableWidget();self.scenario_table.setColumnCount(5);self.scenario_table.setHorizontalHeaderLabels(['网表','器件','失效情景','建议检测路径','性质'])
        rows=[]
        for board,ref in inventory():
            for mode in failure_modes(ref):rows.append((board,ref,mode,detection_path(ref,board),'软件建模 / 仪器检查'))
        self.scenario_table.setRowCount(len(rows))
        for i,row in enumerate(rows):
            for j,val in enumerate(row):self.scenario_table.setItem(i,j,QtWidgets.QTableWidgetItem(val))
        self.scenario_table.horizontalHeader().setStretchLastSection(True);sv.addWidget(self.scenario_table)
        label=QtWidgets.QLabel(f'已覆盖 {len(inventory())} 个网表器件、{len(rows)} 个典型失效情景。通用阻容/焊点故障必须依靠仪器定位，软件不能真实改变器件状态。')
        sv.addWidget(label);tabs.addTab(scenarios,'器件失效情景')
        self.component_button=QtWidgets.QPushButton('打开逐器件排查清单…')
        self.component_button.clicked.connect(self.open_component_checklist)
        sv.addWidget(self.component_button)
        btns=QtWidgets.QDialogButtonBox();self.export=btns.addButton('导出检查报告…',QtWidgets.QDialogButtonBox.ActionRole);btns.addButton(QtWidgets.QDialogButtonBox.Close)
        btns.rejected.connect(self.reject);btns.accepted.connect(self.accept);self.export.clicked.connect(self.export_report);layout.addWidget(btns)
        self.live_timer=QtCore.QTimer(self);self.live_timer.setInterval(500);self.live_timer.timeout.connect(self.refresh_live);self.live_timer.start()
        self.component_results={}
    def open_component_checklist(self):
        dialog=ComponentChecklistDialog(self,self.component_results)
        dialog.exec_();self.component_results=dialog.results
    def refresh_live(self):
        p=self.parent()
        if p is not None:
            self.latest=dict(p.worker.latest) if getattr(p,'_fresh',lambda:False)() else {}
            self.params=dict(getattr(p,'param_cache',{}));self.connected=bool(getattr(p,'_connected',False))
            self.fresh=bool(getattr(p,'_fresh',lambda:False)())
        rows=live_checks(self.latest,self.params,self.connected,self.fresh)
        self.live_table.setRowCount(len(rows))
        for i,row in enumerate(rows):
            for j,value in enumerate(row):self.live_table.setItem(i,j,QtWidgets.QTableWidgetItem(str(value)))
    def export_report(self):
        self.refresh_live()
        path,_=QtWidgets.QFileDialog.getSaveFileName(self,'导出自检报告','crowerfoc_diagnostic.json','JSON (*.json)')
        if not path:return
        data={'time':time.strftime('%Y-%m-%d %H:%M:%S'),'connected':self.connected,'telemetry_fresh':self.fresh,
              'live_checks':[dict(item=name,result=result,detail=detail) for name,result,detail in live_checks(self.latest,self.params,self.connected,self.fresh)],
              'component_checklist':list(self.component_results.values()),
              'netlist_scenarios':[dict(board=b,reference=r,failure=m,detection=d,kind=k) for b,r,m,d,k in (tuple(self.scenario_table.item(i,j).text() for j in range(5)) for i in range(self.scenario_table.rowCount()))]}
        Path(path).write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding='utf-8')
        QtWidgets.QMessageBox.information(self,'报告已导出',path)


class ComponentChecklistDialog(QtWidgets.QDialog):
    """Sequential hands-on inspection checklist based on the two netlists."""
    STATES=('未检查','正常','异常','不适用')
    def __init__(self,parent=None,results=None):
        super().__init__(parent);self.setWindowTitle('逐器件排查清单');self.resize(920,650)
        self.results=dict(results or {});self._loading=False;self._loaded_index=None
        self.parts=inventory()
        root=QtWidgets.QVBoxLayout(self)
        warning=QtWidgets.QLabel('按网表逐个检查器件本体与焊点。此向导不会电气注入开路/短路；断电连续性检查与带电测量请按对应安全步骤进行。每项记录结果和备注，方便导出复查。')
        warning.setWordWrap(True);root.addWidget(warning)
        row=QtWidgets.QHBoxLayout();self.list=QtWidgets.QListWidget();self.list.setMinimumWidth(210)
        for board,ref in self.parts:self.list.addItem(f'{board} · {ref}')
        row.addWidget(self.list,1)
        right=QtWidgets.QWidget();form=QtWidgets.QFormLayout(right)
        self.title=QtWidgets.QLabel();self.title.setStyleSheet('font-size:18px;font-weight:bold')
        self.failures=QtWidgets.QLabel();self.failures.setWordWrap(True)
        self.path=QtWidgets.QLabel();self.path.setWordWrap(True)
        self.state=QtWidgets.QComboBox();self.state.addItems(self.STATES)
        self.notes=QtWidgets.QTextEdit();self.notes.setPlaceholderText('记录测量值、焊点外观、仪器和异常现象…')
        form.addRow(self.title);form.addRow('需覆盖的失效模式',self.failures);form.addRow('建议检查方法',self.path)
        form.addRow('检查结果',self.state);form.addRow('检查记录',self.notes)
        row.addWidget(right,3);root.addLayout(row,1)
        buttons=QtWidgets.QHBoxLayout();self.prev=QtWidgets.QPushButton('上一个');self.next=QtWidgets.QPushButton('保存并下一个')
        self.export=QtWidgets.QPushButton('导出检查记录…');self.close_button=QtWidgets.QPushButton('完成')
        for b in (self.prev,self.next,self.export,self.close_button):buttons.addWidget(b)
        root.addLayout(buttons)
        self.list.currentRowChanged.connect(self.load_part)
        self.prev.clicked.connect(lambda:self.list.setCurrentRow(max(0,self.list.currentRow()-1)))
        self.next.clicked.connect(lambda:self.list.setCurrentRow(min(len(self.parts)-1,self.list.currentRow()+1)))
        self.close_button.clicked.connect(self.finish);self.export.clicked.connect(self.export_results)
        if self.parts:self.list.setCurrentRow(0)
    def _key(self,index):
        board,ref=self.parts[index];return f'{board}/{ref}'
    def save_current(self):
        i=self._loaded_index
        if i is None or i<0:return
        self.results[self._key(i)]={'board':self.parts[i][0],'reference':self.parts[i][1],
                                    'status':self.state.currentText(),'notes':self.notes.toPlainText(),
                                    'failure_modes':list(failure_modes(self.parts[i][1])),
                                    'detection_path':detection_path(self.parts[i][1],self.parts[i][0])}
    def load_part(self,index):
        if index<0 or index>=len(self.parts):return
        self.save_current();board,ref=self.parts[index];saved=self.results.get(self._key(index),{})
        self._loading=True
        self.title.setText(f'{index+1}/{len(self.parts)} · {board} · {ref}')
        self.failures.setText('；'.join(failure_modes(ref)))
        self.path.setText(detection_path(ref,board))
        self.state.setCurrentIndex(max(0,self.state.findText(saved.get('status','未检查'))))
        self.notes.setPlainText(saved.get('notes',''))
        self._loading=False
        self._loaded_index=index
    def export_results(self):
        self.save_current()
        path,_=QtWidgets.QFileDialog.getSaveFileName(self,'导出逐器件检查记录','crowerfoc_component_check.json','JSON (*.json)')
        if path:
            Path(path).write_text(json.dumps({'time':time.strftime('%Y-%m-%d %H:%M:%S'),
                                              'completed':len(self.results),'total':len(self.parts),
                                              'components':list(self.results.values())},ensure_ascii=False,indent=2),encoding='utf-8')
            QtWidgets.QMessageBox.information(self,'检查记录已导出',path)
    def finish(self):
        self.save_current();self.accept()
    def closeEvent(self,event):
        self.save_current();super().closeEvent(event)
