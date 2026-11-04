#!/usr/bin/env python3
"""
CrowerFOC 上位机（界面中文）。

与固件 crowerfoc_proto.h 保持一致：
  帧：AA 55 | 类型 | 长度 | 载荷 | crc8(类型..载荷)
  设备->主机：0x01 遥测 0x02 应答 0x03 参数 0x04 日志 0x05 信息
  主机->设备：0x81 设置 0x82 读取 0x83 状态 0x84 设定值 0x85 保存 0x86 清故障 0x87 重启
"""
import sys
import os
import time
import struct
import math
import json
from pathlib import Path
from collections import deque
from axis_extensions import configuration_document, parse_configuration
import numpy as np

import serial
import serial.tools.list_ports
from PyQt5 import QtCore, QtGui, QtWidgets
import pyqtgraph as pg

from crowerfoc_protocol import *
from crowerfoc_transport import SerialWorker, find_port
from host_tools import FaultDetailWidget, CanDebugWidget
from motor_music import MotorMusicWidget


# ----------------------------------------------------------------- 主窗口
class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("CrowerFOC 电机驱动上位机")
        self.resize(1560, 980)
        self._responsive_layout_applied = False
        self.worker = None
        self._connected = False
        self._closing = False
        self._stale = False
        self.param_cache = {}
        self._writes = {}
        self._reads = set()
        self._pending_state = None
        self._last_state = None
        self._last_fault = 0
        self._cal_started = None
        self._cal_seen = False
        self._active_cal_kind = None
        self._cal_cancelled = False
        self._cal_last_phase = None
        self._diag_due = 0.0
        self._result_wait = set()
        self._result_data = {}
        self._result_deadline = 0.0
        self._dialogs = []
        self._log_history = deque(maxlen=5000)
        self._component_results = {}
        self._bench_active = False
        self._appearance = None
        self._save_pending = None
        self._restore_steps = deque()
        self._restore_pending = False
        self._restore_deadline = 0
        self.window = WINDOW
        self._configured_telem_hz = None

        self._build_toolbar()
        self._build_controls()
        self._torque_drag = False
        self._torque_timer = QtCore.QTimer(self)
        self._torque_timer.setInterval(50)
        self._torque_timer.timeout.connect(self._send_torque_slider)
        self._build_plots()
        self._build_params()
        self._build_layout()
        self.refresh_ports()
        self.setStyleSheet("""
            QGroupBox { font-weight:600; margin-top:10px; padding-top:12px; }
            QPushButton { padding:6px 9px; }
            QPushButton:disabled, QComboBox:disabled, QDoubleSpinBox:disabled,
            QLineEdit:disabled { color:#888; background:#e7e7e7; border:1px solid #ccc; }
            QProgressBar { min-height:18px; text-align:center; }
        """)
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.tick)
        self.timer.start(20)
        self._update_controls()

    def changeEvent(self, event):
        if event.type() in (QtCore.QEvent.WindowDeactivate, QtCore.QEvent.ApplicationDeactivate):
            if getattr(self,"_torque_drag",False):self._torque_slider_released()
        super().changeEvent(event)

    def showEvent(self, event):
        super().showEvent(event)
        if not self._responsive_layout_applied:
            QtCore.QTimer.singleShot(0, self._apply_responsive_layout)

    def _apply_responsive_layout(self):
        """Fit the monitor, workspace, and log to the active screen."""
        screen = self.screen() or QtWidgets.QApplication.primaryScreen()
        if screen is None:
            return
        area = screen.availableGeometry()
        height, width = max(600, area.height()), max(900, area.width())
        compact = height < 700
        compact_ui = height < 850
        monitor_h = (105 if compact else
                     max(145, min(260, int(height * (0.20 if height < 850 else 0.33)))))
        log_h = max(70, min(130, int(height * 0.105)))
        self.live_monitor.setMinimumHeight(0)
        for plot in (self.pw_cur_live, self.pw_dq_live, self.pw_enc_live):
            plot.setMinimumSize(45 if compact else 90, 40 if compact else 70)
        self.main_splitter.setSizes([monitor_h, max(220, height - monitor_h - log_h - 150)])
        if compact:
            self.log_dock.hide()
        else:
            self.log_dock.show()
            self.resizeDocks([self.log_dock], [log_h], QtCore.Qt.Vertical)
        if compact_ui:
            self.control_panel.setStyleSheet("QWidget { font-size:9pt; } QGroupBox { margin-top:2px; padding-top:2px; } QPushButton { padding:3px 4px; min-height:22px; }")
            self.status_panel.setStyleSheet("QWidget { font-size:9pt; }")
        else:
            self.control_panel.setStyleSheet("")
            self.status_panel.setStyleSheet("")
        status_width = max(220 if width < 1200 else 330,
                           min(470, int(width * (0.24 if width < 1200 else 0.28))))
        self.status_panel.setMaximumWidth(status_width)
        self.status_panel.setMinimumWidth(0)
        self.control_splitter.setSizes([max(450, width - status_width - 60), status_width])
        self._responsive_layout_applied = True

    # ---------- 顶栏
    def _build_toolbar(self):
        self.cb_port = QtWidgets.QComboBox()
        self.cb_port.setMinimumWidth(120)
        self.cb_baud = QtWidgets.QComboBox()
        for baud in (115200, 460800, 921600):
            self.cb_baud.addItem(str(baud), baud)
        self.cb_baud.setCurrentIndex(self.cb_baud.findData(115200))
        self.cb_port.activated.connect(self._port_baud_default)
        self.btn_refresh = QtWidgets.QPushButton("刷新")
        self.btn_conn = QtWidgets.QPushButton("连接")
        self.btn_conn.setMinimumWidth(90)
        self.lb_conn = QtWidgets.QLabel("未连接")
        self.lb_rate = QtWidgets.QLabel("速率 --")
        self.lb_vbus = QtWidgets.QLabel("母线 --")
        self.lb_state = QtWidgets.QLabel("状态 --")
        self.lb_fault = QtWidgets.QLabel("故障 --")
        self.lb_encoder = QtWidgets.QLabel("编码器1（电机侧）-- · 编码器2（输出轴）--")
        self.lb_mapping = QtWidgets.QLabel("换相 --")
        self.lb_thermal = QtWidgets.QLabel("温度 -- · 控制来源 --")
        lay = QtWidgets.QHBoxLayout()
        for w in (QtWidgets.QLabel("端口"), self.cb_port, QtWidgets.QLabel("波特率"), self.cb_baud, self.btn_refresh,
                  self.btn_conn, self.lb_conn, self.lb_rate, self.lb_vbus):
            lay.addWidget(w)
        lay.addStretch(1)
        w = QtWidgets.QWidget()
        outer = QtWidgets.QVBoxLayout(w)
        outer.setContentsMargins(5, 3, 5, 3)
        outer.setSpacing(2)
        outer.addLayout(lay)
        status_row = QtWidgets.QHBoxLayout()
        status_row.setSpacing(8)
        for label in (self.lb_state, self.lb_encoder, self.lb_mapping, self.lb_fault):
            status_row.addWidget(label)
        status_row.addStretch()
        outer.addLayout(status_row)
        self.state_banner = QtWidgets.QLabel("未连接 · 连接后自动同步设备状态与参数")
        self.state_banner.setWordWrap(True)
        self.state_banner.setMinimumHeight(24)
        outer.addWidget(self.state_banner)
        self.toolbar_widget = w
        self.btn_refresh.clicked.connect(self.refresh_ports)
        self.btn_conn.clicked.connect(self.toggle_conn)

    # ---------- 控制面板（左）
    def _build_controls(self):
        g = QtWidgets.QGroupBox("轴控制")
        v = QtWidgets.QGridLayout(g)
        v.setContentsMargins(4, 3, 4, 3)
        v.setHorizontalSpacing(4)
        v.setVerticalSpacing(2)

        state_box = QtWidgets.QGroupBox("驱动状态")
        state_layout = QtWidgets.QHBoxLayout(state_box)
        state_layout.setContentsMargins(6, 6, 6, 6)
        row = QtWidgets.QHBoxLayout()
        self.bt_idle = QtWidgets.QPushButton("停止 / 空闲")
        self.bt_idle.setStyleSheet("font-weight:bold; color:#c9302c")
        self.bt_closed = QtWidgets.QPushButton("进入闭环")
        self.bt_closed.setToolTip("以零指令进入闭环；确认设备状态后再发送运动指令")
        for b in (self.bt_idle, self.bt_closed):
            row.addWidget(b)
        state_layout.addLayout(row, 1)
        self.bt_idle.clicked.connect(lambda: self.cmd_state(1))
        self.bt_closed.clicked.connect(lambda: self.cmd_state(3))
        self.bt_cal = QtWidgets.QPushButton("完整标定")
        self.bt_cal.setToolTip("新版固件固定5A校准，计划约14.5秒；实际电流与时长以设备回读为准。轴需自由转动。")
        v.addWidget(state_box, 0, 0)
        cal_box = QtWidgets.QGroupBox("电机标定")
        cal_layout = QtWidgets.QGridLayout(cal_box)
        cal_layout.setContentsMargins(4, 3, 4, 3)
        cal_layout.setVerticalSpacing(2)
        cal_layout.setHorizontalSpacing(4)
        self.bt_cal.clicked.connect(lambda: self.cmd_state(2))
        self.bt_bw = QtWidgets.QPushButton("电流环扫频 / 波特图")
        self.bt_bw.setToolTip("完成标定后执行独立d轴正弦扫频，查看多频点幅相响应")
        self.bt_bw.clicked.connect(self.cmd_bw_sweep)
        self.lb_progress = QtWidgets.QLabel("连接设备后读取参数，再进行完整标定")
        self.lb_progress.setWordWrap(False)
        self.progress = QtWidgets.QProgressBar()
        self.progress.setRange(0, 100)
        self.progress.setMaximumHeight(14)
        cal_layout.addWidget(self.bt_cal, 0, 0)
        cal_layout.addWidget(self.bt_bw, 0, 1)
        cal_layout.addWidget(self.progress, 1, 0, 1, 2)
        cal_layout.addWidget(self.lb_progress, 2, 0, 1, 2)
        self.lb_cal_hint = QtWidgets.QLabel("轴需自由转动；固件使用5A校准。结果见弹窗。")
        self.lb_cal_hint.setWordWrap(False)
        self.lb_cal_hint.setStyleSheet("color:#6f4b00;")
        cal_layout.addWidget(self.lb_cal_hint, 3, 0, 1, 2)

        limits = QtWidgets.QFormLayout()
        limits.setVerticalSpacing(2)
        self.sp_limit_cur = QtWidgets.QDoubleSpinBox()
        self.sp_limit_cur.setRange(0.05, 40.0)
        self.sp_limit_cur.setDecimals(2)
        self.sp_limit_cur.setValue(40.0)
        self.sp_limit_vel = QtWidgets.QDoubleSpinBox()
        self.sp_limit_vel.setRange(0.1, 2100.0)
        self.sp_limit_vel.setDecimals(2)
        self.sp_limit_vel.setValue(230.0)
        self.sp_cal_cur = QtWidgets.QDoubleSpinBox()
        self.sp_cal_cur.setRange(0.05, 20.0)
        self.sp_cal_cur.setDecimals(2)
        self.sp_cal_cur.setValue(5.0)
        self.sp_cal_cur.setReadOnly(True)
        self.sp_cal_cur.setButtonSymbols(QtWidgets.QAbstractSpinBox.NoButtons)
        self.sp_cal_cur.setEnabled(False)
        self.sp_cal_cur.setToolTip("由固件固定配置。新版独立于正常运行限流；旧版需更新固件才能使用新校准流程。")
        limits.addRow("电流限制 / A", self.sp_limit_cur)
        self.sp_limit_vel.setToolTip("无输出轴编码器时按电机rpm限速；健康的输出轴编码器启用后按输出轴rpm限速。")
        self.lb_speed_limit = QtWidgets.QLabel("速度指令限制 / rpm")
        limits.addRow(self.lb_speed_limit, self.sp_limit_vel)
        limits.addRow("固件标定电流 / A", self.sp_cal_cur)
        self.bt_limits = QtWidgets.QPushButton("应用运行限制")
        self.bt_limits.setToolTip("写入后自动回读，确认成功再允许启动")
        limits_box = QtWidgets.QGroupBox("运行限制")
        limits_layout = QtWidgets.QVBoxLayout(limits_box)
        limits_layout.setContentsMargins(4, 3, 4, 3)
        limits_layout.setSpacing(2)
        limits_layout.addLayout(limits)
        limits_layout.addWidget(self.bt_limits)
        v.addWidget(cal_box, 0, 1)
        v.addWidget(limits_box, 1, 0)
        self.bt_limits.clicked.connect(self.apply_limits)

        self.cb_mode = QtWidgets.QComboBox()
        self.cb_mode.addItem("Iq电流（A）", 0)
        self.cb_mode.addItem("速度（rpm）", 1)
        self.cb_mode.addItem("位置（°）", 2)
        self.cb_mode.setToolTip("空载持续Iq会使电机持续加速；保持恒速请用速度模式。运行时以速度指令限制限制速度模式目标。")
        self.cb_mode.currentIndexChanged.connect(self.change_mode)

        self.cb_inmode = QtWidgets.QComboBox()
        self.cb_inmode.addItem("直通",1)
        self.cb_inmode.currentIndexChanged.connect(self.change_input_mode)

        enc_box = QtWidgets.QGroupBox("输出轴第二编码器")
        enc_form = QtWidgets.QFormLayout(enc_box)
        enc_form.setContentsMargins(4, 3, 4, 3)
        enc_form.setVerticalSpacing(2)
        self.ck_encoder2 = QtWidgets.QCheckBox("启用第二编码器反馈")
        self.lb_encoder2_health = QtWidgets.QLabel("未连接 / 未启用")
        self.lb_gear_ratio = QtWidgets.QLabel("实测传动比：--")
        enc_form.addRow(self.ck_encoder2)
        enc_form.addRow("健康状态", self.lb_encoder2_health)
        enc_form.addRow("电机角 / 输出角", self.lb_gear_ratio)
        self.ck_encoder2.toggled.connect(self._set_encoder2_enabled)
        self._encoder2_poll_at = 0.0

        self.target_group = QtWidgets.QGroupBox("运动指令 · 闭环后启用")
        target_layout = QtWidgets.QGridLayout(self.target_group)
        target_layout.setContentsMargins(4, 3, 4, 3)
        target_layout.setVerticalSpacing(2)
        target_layout.setHorizontalSpacing(4)
        self.lb_setpoint = QtWidgets.QLabel("Iq电流设定值 / A")
        target_layout.addWidget(self.lb_setpoint, 0, 0)
        self.cb_torque_input = QtWidgets.QComboBox()
        self.cb_torque_input.addItem("数值输入", "numeric")
        self.cb_torque_input.addItem("按住滑块输出（松开归零）", "hold")
        target_layout.addWidget(self.cb_torque_input, 0, 1)
        srow = QtWidgets.QHBoxLayout()
        self.sp_set = QtWidgets.QDoubleSpinBox()
        self.sp_set.setRange(-1, 1)
        self.sp_set.setDecimals(3)
        self.bt_set = QtWidgets.QPushButton("发送指令")
        srow.addWidget(self.sp_set)
        srow.addWidget(self.bt_set)
        target_layout.addLayout(srow, 1, 0, 1, 2)
        self.slider_box = QtWidgets.QWidget()
        slider_layout = QtWidgets.QVBoxLayout(self.slider_box)
        slider_layout.setContentsMargins(0, 0, 0, 0)
        self.lb_torque_slider = QtWidgets.QLabel("按住并拖动：0.00 A")
        self.sl_torque = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.sl_torque.setRange(-1000, 1000)
        self.sl_torque.setValue(0)
        slider_layout.addWidget(self.lb_torque_slider)
        slider_layout.addWidget(self.sl_torque)
        target_layout.addWidget(self.slider_box, 2, 0, 1, 2)
        self.slider_box.hide()
        self.cb_torque_input.currentIndexChanged.connect(self._torque_input_changed)
        self.sl_torque.sliderPressed.connect(self._torque_slider_pressed)
        self.sl_torque.sliderReleased.connect(self._torque_slider_released)
        self.sl_torque.valueChanged.connect(lambda _v: self._update_torque_slider_label())
        self.bt_set.clicked.connect(lambda: self.send_setpoint(self.sp_set.value()))

        command_box = QtWidgets.QGroupBox("控制指令")
        command_layout = QtWidgets.QGridLayout(command_box)
        command_layout.setContentsMargins(4, 3, 4, 3)
        command_layout.setVerticalSpacing(2)
        command_layout.addWidget(QtWidgets.QLabel("模式"), 0, 0)
        command_layout.addWidget(self.cb_mode, 0, 1)
        command_layout.addWidget(QtWidgets.QLabel("输入规划"), 0, 2)
        command_layout.addWidget(self.cb_inmode, 0, 3)
        command_layout.addWidget(self.target_group, 1, 0, 1, 4)
        v.addWidget(command_box, 1, 1)
        v.addWidget(enc_box, 2, 0)

        self.rb_legacy = QtWidgets.QCheckBox("展开开环 V/f 调试")
        bench_box = QtWidgets.QGroupBox("调试 / 维护")
        bench_layout_outer = QtWidgets.QVBoxLayout(bench_box)
        bench_layout_outer.setContentsMargins(4, 3, 4, 3)
        bench_layout_outer.setSpacing(2)
        bench_layout_outer.addWidget(self.rb_legacy)
        self.rb_legacy.toggled.connect(self.toggle_bench)
        self.bench_panel = QtWidgets.QWidget()
        bench_layout = QtWidgets.QVBoxLayout(self.bench_panel)
        bench_layout.setContentsMargins(0, 0, 0, 0)
        self.sl_mod = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.sl_mod.setRange(0, 200)
        self.lb_mod = QtWidgets.QLabel("调制 0")
        self.sl_freq = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.sl_freq.setRange(1, 200)
        self.sl_freq.setValue(20)
        self.lb_freq = QtWidgets.QLabel("频率 2.0Hz")
        for w in (self.lb_mod, self.sl_mod, self.lb_freq, self.sl_freq):
            bench_layout.addWidget(w)
        self.sl_mod.valueChanged.connect(lambda x: self.lb_mod.setText("调制 %d（尚未应用）" % x))
        self.sl_freq.valueChanged.connect(lambda x: self.lb_freq.setText("电频率 %.1fHz" % (x / 10.0)))
        self.bt_bench = QtWidgets.QPushButton("应用开环 V/f")
        bench_layout.addWidget(self.bt_bench)
        bench_layout_outer.addWidget(self.bench_panel)
        self.bench_panel.hide()
        self.rb_legacy.toggled.connect(self.bench_panel.setVisible)
        self.bt_bench.clicked.connect(self.apply_bench)

        row2 = QtWidgets.QHBoxLayout()
        self.bt_clear = QtWidgets.QPushButton("清故障")
        self.bt_clear.setToolTip("清除设备当前可清除的锁存故障")
        self.bt_save = QtWidgets.QPushButton("保存")
        self.bt_save.setEnabled(False)
        self.bt_save.setToolTip("空闲时保存配置与有效校准；设备校验成功后才提示完成")
        self.bt_reboot = QtWidgets.QPushButton("重启")
        row2.addWidget(self.bt_clear)
        row2.addWidget(self.bt_reboot)
        row2.addWidget(self.bt_save)
        self.bt_save.clicked.connect(self.save_config)
        self.bt_clear.clicked.connect(self.clear_faults)
        self.bt_reboot.clicked.connect(self.reboot)
        self.bt_backup=QtWidgets.QPushButton("导出配置")
        self.bt_restore=QtWidgets.QPushButton("导入配置")
        self.bt_backup.setToolTip("导出当前配置文件")
        self.bt_restore.setToolTip("从配置文件导入参数")
        self.bt_backup.clicked.connect(self.backup_config);self.bt_restore.clicked.connect(self.restore_config)
        self.bt_can=QtWidgets.QPushButton("CAN许可")
        self.bt_can.setToolTip("空闲、无故障、校准有效时许可 MIT CAN 控制；停止或故障后需要重新许可")
        self.bt_can.clicked.connect(lambda:self.do_set(0x0802,1))
        self.bt_diagnostics=QtWidgets.QPushButton("器件自检")
        self.bt_diagnostics.setToolTip("运行设备自检并逐项检查网表相关故障")
        self.bt_diagnostics.clicked.connect(self.open_diagnostics)
        service_layout = QtWidgets.QGridLayout()
        service_layout.setContentsMargins(0, 0, 0, 0)
        service_layout.setSpacing(2)
        for col, button in enumerate((self.bt_clear, self.bt_reboot, self.bt_save, self.bt_backup)):
            service_layout.addWidget(button, 0, col)
        for col, button in enumerate((self.bt_restore, self.bt_can, self.bt_diagnostics)):
            service_layout.addWidget(button, 1, col)
        bench_layout_outer.addLayout(service_layout)
        v.addWidget(bench_box, 2, 1)
        note = QtWidgets.QLabel("提示：校准参数需保存到 Flash；空闲态 d/q 显示最后一次计算值。")
        note.setWordWrap(False)
        note.setMaximumHeight(22)
        note.hide()
        v.setColumnStretch(0, 1)
        v.setColumnStretch(1, 1)
        self.control_panel = g
        g.setMinimumWidth(0)
        g.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Preferred)
        for box in (state_box, cal_box, limits_box, enc_box, self.target_group,
                    command_box, bench_box):
            box.setMinimumWidth(0)
            box.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Preferred)

        status = QtWidgets.QGroupBox("实时状态")
        status_grid = QtWidgets.QGridLayout(status)
        status_grid.setContentsMargins(4, 4, 4, 4)
        status_grid.setHorizontalSpacing(8)
        status_grid.setVerticalSpacing(3)
        self.readout_labels = {}
        self.readout_title_labels = {}
        readouts = [
            ("设备", "device"), ("轴状态", "axis"),
            ("故障", "fault"), ("母线", "bus"),
            ("相电流 A / B / C", "phase"), ("Id / Iq", "dq"),
            ("ENC1 机械角 / 电角度", "angle"), ("ENC1 电机转速", "speed"),
            ("ENC1 累计机械位置", "position"), ("ENC2 绝对机械角", "enc2_angle"),
            ("ENC2 输出轴转速", "enc2_speed"), ("ENC2 累计机械位置", "enc2_position"),
            ("ENC1 / ENC2 实测传动比", "ratio"), ("采样速率", "rate"),
        ]
        for index, (caption, key) in enumerate(readouts):
            row, col = divmod(index, 2)
            cell = QtWidgets.QWidget()
            cell_layout = QtWidgets.QVBoxLayout(cell)
            cell_layout.setContentsMargins(2, 0, 2, 0)
            cell_layout.setSpacing(0)
            title = QtWidgets.QLabel(caption)
            title.setStyleSheet("color:#687386; font-size:9pt")
            value = QtWidgets.QLabel("--")
            value.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
            value.setStyleSheet("font-size:11pt; font-weight:600")
            cell_layout.addWidget(title)
            cell_layout.addWidget(value)
            status_grid.addWidget(cell, row, col)
            self.readout_labels[key] = value
            self.readout_title_labels[key] = title
        self.lb_readout_note = QtWidgets.QLabel("连接设备后显示遥测；断开时控制项不可操作。")
        self.lb_readout_note.setWordWrap(True)
        self.lb_readout_note.setStyleSheet("padding:3px; background:#eef2f6; color:#48566a")
        status_grid.addWidget(self.lb_readout_note, (len(readouts)+1)//2, 0, 1, 2)
        status.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Preferred)

        self.control_splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        self.control_splitter.setChildrenCollapsible(False)
        self.control_splitter.setMinimumWidth(0)
        self.control_splitter.addWidget(g)
        self.control_splitter.addWidget(status)
        self.control_splitter.setStretchFactor(0, 1)
        self.control_splitter.setStretchFactor(1, 0)
        self.control_splitter.setSizes([1200, 430])
        status.setMaximumWidth(470)
        self.status_panel = status
        page = QtWidgets.QWidget()
        page_layout = QtWidgets.QHBoxLayout(page)
        page_layout.setContentsMargins(0, 0, 0, 0)
        page_layout.addWidget(self.control_splitter)
        page.setMinimumSize(0, 0)
        page.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Ignored)
        self.control_page = page

    # ---------- 曲线（中）
    def _mk(self, title, ylabel, fields, colors):
        pw = pg.PlotWidget(title=title)
        pw.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        pw.addLegend()
        pw.setLabel("left", ylabel)
        # Keep amperes/degrees/rpm in the units stated by the label. AxisItem's
        # automatic SI prefix otherwise shows e.g. 200 × 0.001 beside “A”.
        pw.getAxis("left").enableAutoSIPrefix(False)
        pw.setLabel("bottom", "时间 / s")
        pw.showGrid(x=True, y=True, alpha=0.3)
        curves = {}
        for f, c in zip(fields, colors):
            # Time-ordered trace: no point markers and no downsampling. This keeps
            # normal-rate USB telemetry legible without drawing per-sample spikes.
            cu = pw.plot(pen=pg.mkPen(c, width=1.25), name=f, connect="finite")
            cu.setClipToView(True)
            curves[f] = cu
        return pw, curves

    def _build_plots(self):
        pg.setConfigOptions(antialias=False, background="k", foreground="w")
        self.pw_cur, self.c_cur = self._mk(
            "三相电流", "电流 / A", ["ia", "ib", "ic"],
            ["#ff5555", "#55ff55", "#5599ff"])
        self.cb_current_view = QtWidgets.QComboBox()
        self.cb_current_view.addItems(["电流环输入（处理后）", "原始采样（保留尖峰）"])
        self.lb_current_view = QtWidgets.QLabel("实时曲线按设备遥测频率更新；电流环内部采样为25kHz")
        self.lb_current_view.setWordWrap(True)
        self._live_plot_connected = None
        current_tab = QtWidgets.QWidget()
        current_layout = QtWidgets.QVBoxLayout(current_tab)
        current_layout.setContentsMargins(0, 0, 0, 0)
        current_layout.addWidget(self.pw_cur, 1)
        self.pw_dq, self.c_dq = self._mk(
            "d/q 轴电流", "电流 / A", ["id", "iq"], ["#ffcc55", "#cc66ff"])
        self.pw_mot, self.c_mot = self._mk(
            "ENC1 · 电机机械角 / 电角度", "角度 / °", ["enc1_ang", "enc1_elec"], ["#55ddff", "#ff88aa"])
        self.pw_vel, self.c_vel = self._mk("ENC1 · 电机机械速度", "rpm", ["enc1_vel"], ["#ffcc55"])
        self.pw_pos, self.c_pos = self._mk("ENC1 · 累计机械位置", "°", ["enc1_pos"], ["#99ccff"])
        self.pw_mot2, self.c_mot2 = self._mk("ENC2 · 输出轴绝对机械角", "角度 / °", ["enc2_ang"], ["#66eebb"])
        self.pw_vel2, self.c_vel2 = self._mk("ENC2 · 输出轴机械速度", "rpm", ["enc2_vel"], ["#ffcc55"])
        self.pw_pos2, self.c_pos2 = self._mk("ENC2 · 累计机械位置", "°", ["enc2_pos"], ["#99ccff"])
        motion = QtWidgets.QWidget()
        motion_layout = QtWidgets.QGridLayout(motion)
        motion_layout.setContentsMargins(0, 0, 0, 0)
        for row, plots in enumerate(((self.pw_mot,self.pw_mot2),(self.pw_vel,self.pw_vel2),(self.pw_pos,self.pw_pos2))):
            for col, plot in enumerate(plots):motion_layout.addWidget(plot,row,col)
            motion_layout.setRowStretch(row,1)
        motion_layout.setColumnStretch(0,1);motion_layout.setColumnStretch(1,1)
        self.pw_duty, self.c_duty = self._mk(
            "PWM 占空比", "占空比 / %", ["da", "db", "dc"],
            ["#ff5555", "#55ff55", "#5599ff"])

        self.pw_vec = pg.PlotWidget(title="电流矢量 (α-β)")
        self.pw_vec.setAspectLocked(True)
        self.pw_vec.showGrid(x=True, y=True, alpha=0.3)
        self.pw_vec.setLabel("left", "β / mA")
        self.pw_vec.setLabel("bottom", "α / mA")
        self.v_traj = self.pw_vec.plot(pen=pg.mkPen("#ffcc55", width=1))
        self.v_now = self.pw_vec.plot(pen=None, symbol="o", symbolSize=9,
                                      symbolBrush="#ffffff", symbolPen=None)

        # Duplicate the three essential plots in a compact, permanently visible
        # monitor. Keeping independent PlotWidgets lets the Curves workspace
        # retain its full-size charts while every other page shows live data.
        self.pw_cur_live, self.c_cur_live = self._mk(
            "三相电流", "电流 / A", ["ia", "ib", "ic"],
            ["#ff5555", "#55ff55", "#5599ff"])
        self.pw_dq_live, self.c_dq_live = self._mk(
            "d/q 轴电流", "电流 / A", ["id", "iq"], ["#ffcc55", "#cc66ff"])
        self.pw_enc_live, self.c_enc_live = self._mk(
            "ENC1机械角 / 电角度 · ENC2机械角", "角度 / °", ["ang", "ele", "enc2_ang"], ["#55ddff", "#ff88aa", "#66eebb"])
        for plot in (self.pw_cur_live, self.pw_dq_live, self.pw_enc_live):
            plot.setMinimumSize(90, 70)
        self.live_splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        self.live_splitter.setChildrenCollapsible(False)
        for plot in (self.pw_cur_live, self.pw_dq_live, self.pw_enc_live):
            self.live_splitter.addWidget(plot)
            self.live_splitter.setStretchFactor(self.live_splitter.count() - 1, 1)
        self.live_monitor = QtWidgets.QGroupBox("实时监视 · 所有界面常驻")
        monitor_layout = QtWidgets.QVBoxLayout(self.live_monitor)
        monitor_layout.setContentsMargins(6, 8, 6, 6)
        monitor_header = QtWidgets.QHBoxLayout()
        monitor_header.addWidget(self.cb_current_view)
        monitor_header.addWidget(self.lb_current_view, 1)
        monitor_layout.addLayout(monitor_header)
        monitor_layout.addWidget(self.live_splitter, 1)
        self.live_monitor.setMinimumHeight(0)
        self.live_monitor.setSizePolicy(QtWidgets.QSizePolicy.Expanding,
                                       QtWidgets.QSizePolicy.Expanding)

        tabs = QtWidgets.QTabWidget()
        tabs.addTab(current_tab, "相电流")
        tabs.addTab(self.pw_dq, "d/q")
        tabs.addTab(motion, "运动")
        tabs.addTab(self.pw_duty, "占空比")
        tabs.addTab(self.pw_vec, "矢量")
        self.tabs = tabs

    # ---------- 参数（右）
    def _build_params(self):
        self.tree = QtWidgets.QTreeWidget()
        self.tree.setColumnCount(3)
        self.tree.setHeaderLabels(["参数", "数值", "单位"])
        self.tree.setColumnWidth(0, 170)
        self.tree.setColumnWidth(1, 100)
        self._tree_items = {}
        groups = {}
        for pid, grp, name, unit in PARAMS:
            if grp not in groups:
                it = QtWidgets.QTreeWidgetItem(self.tree, [grp, "", ""])
                it.setExpanded(True)
                groups[grp] = it
            leaf = QtWidgets.QTreeWidgetItem(groups[grp], [name, "", unit])
            leaf.setData(0, QtCore.Qt.UserRole, pid)
            leaf.setToolTip(0, f"{name} · 0x{pid:04X}")
            leaf.setToolTip(2, unit)
            self._tree_items[pid] = leaf
        self.tree.itemDoubleClicked.connect(self.on_param_dblclick)

        g = QtWidgets.QGroupBox("设备参数 · 双击读取")
        v = QtWidgets.QVBoxLayout(g)
        thermal_box = QtWidgets.QGroupBox("温度保护开关")
        thermal_layout = QtWidgets.QVBoxLayout(thermal_box)
        thermal_layout.setContentsMargins(6, 4, 6, 4)
        thermal_layout.setSpacing(2)
        self.ck_fet_temp = QtWidgets.QCheckBox("板载 NTC 过温保护（PA1）")
        self.ck_motor_temp = QtWidgets.QCheckBox("电机 TEMP2 过温保护（PA2；两点 PTC 拟合）")
        self.ck_fet_temp.setToolTip("关闭后板载温度不再降额或触发过温故障；过流、母线电压和其他保护仍有效。")
        self.ck_motor_temp.setToolTip("TEMP2 按实测点 (25℃, 620Ω)、(48℃, 848Ω) 做线性 PTC 换算；48℃以上为外推。开启后会按降额/跳闸阈值动作。")
        thermal_layout.addWidget(self.ck_fet_temp)
        thermal_layout.addWidget(self.ck_motor_temp)
        self.lb_thermal_config = QtWidgets.QLabel("开关写入后立即生效；需要重启后保留时，再点控制页的“保存”。")
        self.lb_thermal_config.setWordWrap(True)
        thermal_layout.addWidget(self.lb_thermal_config)
        v.addWidget(thermal_box)
        self.ck_fet_temp.toggled.connect(lambda enabled: self._set_thermal_protection(0x0437, enabled))
        self.ck_motor_temp.toggled.connect(lambda enabled: self._set_thermal_protection(0x0432, enabled))
        v.addWidget(self.tree)
        row = QtWidgets.QHBoxLayout()
        self.cb_pid = QtWidgets.QComboBox()
        for pid, grp, name, unit in PARAMS:
            if pid in WRITABLE_PARAMS:
                self.cb_pid.addItem("%s.%s" % (grp, name), pid)
        self.ed_param = QtWidgets.QLineEdit()
        self.ed_param.setPlaceholderText("数值")
        self.ed_param.setMinimumWidth(80)
        self.bt_pset = QtWidgets.QPushButton("应用")
        row.addWidget(self.cb_pid)
        row.addWidget(self.ed_param)
        row.addWidget(self.bt_pset)
        v.addLayout(row)
        self.bt_pset.clicked.connect(self.on_param_apply)
        self.cb_pid.currentIndexChanged.connect(self.select_param)
        self.tree.itemClicked.connect(self.select_tree_param)
        self.lb_param_status = QtWidgets.QLabel("未连接")
        self.lb_param_status.setWordWrap(True)
        v.addWidget(self.lb_param_status)
        self.bt_prefresh = QtWidgets.QPushButton("读取全部")
        v.addWidget(self.bt_prefresh)
        self.bt_prefresh.clicked.connect(self.read_all_params)

        g.setMinimumSize(0, 0)
        g.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Ignored)
        self.parameter_page = g

    def _build_layout(self):
        lg = QtWidgets.QPlainTextEdit()
        lg.setReadOnly(True)
        lg.setMaximumBlockCount(500)
        lg.setMinimumHeight(24)
        self.log = lg
        d = QtWidgets.QDockWidget("日志", self)
        d.setWidget(lg)
        d.setFeatures(QtWidgets.QDockWidget.DockWidgetMovable)
        d.setMinimumHeight(48)
        self.log_dock = d
        self.addDockWidget(QtCore.Qt.BottomDockWidgetArea, d)
        view_menu = self.menuBar().addMenu("视图")
        view_menu.addAction(d.toggleViewAction())

        central = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(central)
        v.setContentsMargins(4, 4, 4, 4)
        v.addWidget(self.toolbar_widget)
        self.workspaces = QtWidgets.QTabWidget()
        self.workspaces.addTab(self.control_page, "控制与校准")
        self.workspaces.addTab(self.tabs, "运行曲线")
        self.workspaces.addTab(self.parameter_page, "参数与配置")
        self.fault_page = FaultDetailWidget(self._export_diagnostic_bundle, self)
        self.can_page = CanDebugWidget(self)
        self.workspaces.addTab(self.fault_page, "故障与自检")
        self.workspaces.addTab(self.can_page, "CAN调试")
        self.music_page=MotorMusicWidget(self)
        self.workspaces.addTab(self.music_page,"电机音乐")
        self.workspaces.setMinimumSize(0, 0)
        self.workspaces.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Ignored)
        for index in range(self.workspaces.count()):
            page = self.workspaces.widget(index)
            page.setMinimumSize(0, 0)
            if index != 0:
                page.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Ignored)
        self.main_splitter = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        self.main_splitter.setChildrenCollapsible(False)
        self.main_splitter.addWidget(self.live_monitor)
        self.main_splitter.addWidget(self.workspaces)
        self.main_splitter.setStretchFactor(0, 0)
        self.main_splitter.setStretchFactor(1, 1)
        self.main_splitter.setSizes([300, 650])
        v.addWidget(self.main_splitter, 1)
        self.setCentralWidget(central)
        self.resizeDocks([d], [150], QtCore.Qt.Vertical)

    # ---------- 连接与设备状态
    def refresh_ports(self):
        current = self.cb_port.currentData()
        self.cb_port.clear()
        for p in serial.tools.list_ports.comports():
            self.cb_port.addItem(f"{p.device} · {p.description}", p.device)
        selected = self.cb_port.findData(current)
        if selected < 0:
            selected = self.cb_port.findData(find_port())
        if selected >= 0:
            self.cb_port.setCurrentIndex(selected)
            self._port_baud_default(selected)

    def _port_baud_default(self, index):
        port = self.cb_port.itemData(index)
        preferred = 115200 if port and port == find_port() else 921600
        self.cb_baud.setCurrentIndex(self.cb_baud.findData(preferred))

    def toggle_conn(self):
        if self.worker is not None:
            self._connected = False
            self.btn_conn.setText("正在停机并断开…")
            self.btn_conn.setEnabled(False)
            self.worker.stop()
            self._update_controls()
            return
        port = self.cb_port.currentData()
        if not port:
            self.logmsg("未发现串口，请连接设备后刷新")
            return
        self.param_cache.clear()
        self._writes.clear()
        self._pending_state = None
        self._last_state = None
        self._stale = False
        self._cal_started = None
        self._cal_seen = False
        self.progress.setValue(0)
        for item in self._tree_items.values():
            item.setText(1, "--")
        baudrate = int(self.cb_baud.currentData() or 115200)
        self.worker = SerialWorker(port, baudrate)
        self.worker.status.connect(self.lb_conn.setText)
        self.worker.connected.connect(self.on_connected)
        self.worker.finished.connect(self.on_disconnected)
        self.worker.logline.connect(self.logmsg)
        self.worker.param.connect(self.on_param_reply)
        self.worker.ack.connect(self.on_ack)
        self.worker.stats.connect(self.on_stats)
        self.worker.sample.connect(self._on_sample)
        self.can_page.set_worker(self.worker)
        self.music_page.set_worker(self.worker)
        self.worker.start()
        self.btn_conn.setText("取消连接")
        self.logmsg(f"正在连接 {port}（{baudrate} bit/s），连接后自动读取设备参数")
        self._update_controls()

    def on_connected(self):
        if self.sender() is not self.worker:
            return
        self._connected = True
        self.btn_conn.setText("停止并断开")
        self.read_all_params()
        self.worker.get_param(0x0630)
        self.worker.get_param(0x062A)
        self._update_controls()

    def on_disconnected(self):
        self._torque_slider_released()
        self._save_pending=None;self._restore_pending=False;self._restore_steps.clear()
        if self.sender() is not self.worker:
            return
        if self._cal_seen and not self._closing:
            self.show_result(False, "连接中断，标定结果无法确认")
        self.worker = None
        self.can_page.set_worker(None)
        self.music_page.set_worker(None)
        self._connected = False
        self._pending_state = None
        self._writes.clear()
        self.param_cache.clear()
        self._cal_seen = False
        self._cal_started = None
        self._result_wait.clear()
        self._bench_active = False
        self.btn_conn.setEnabled(True)
        self.btn_conn.setText("连接")
        self.lb_conn.setText("未连接")
        self._configured_telem_hz = None
        for label in (self.lb_state, self.lb_fault, self.lb_vbus, self.lb_rate,
                      self.lb_encoder, self.lb_mapping, self.lb_thermal):
            label.setText("--")
        self.lb_param_status.setText("已断开，设备参数需重新读取")
        self.logmsg("串口已关闭；关闭前已尝试清零指令并切换空闲")
        self._update_controls()
        if self._closing:
            self.close()

    def on_stats(self, fps, total, missed):
        configured = (f"配置 {self._configured_telem_hz:.0f}Hz · "
                      if self._configured_telem_hz is not None else "")
        self.lb_rate.setText(f"遥测 {configured}实收 {fps:.0f}Hz · 帧{total} 丢{missed}")

    def logmsg(self, text):
        self._log_history.append(f"[{time.strftime('%H:%M:%S')}] {text}")
        self.log.appendPlainText("[%s] %s" % (time.strftime("%H:%M:%S"), text))

    def _on_sample(self, sample):
        self.fault_page.on_sample(sample)

    def _diagnostic_snapshot(self):
        worker = self.worker
        return {
            "metadata": {"created": time.strftime("%Y-%m-%d %H:%M:%S"),
                         "port": getattr(worker, "port", None),
                         "host_ui": "CrowerFOC HostUI"},
            "status": dict(worker.latest) if worker else {},
            "parameters": {f"0x{pid:04X}": value for pid, value in self.param_cache.items()},
            "logs": "\n".join(self._log_history),
            "fault_history": list(self.fault_page.history),
            "component_checklist": self._component_results,
            "hardware_scenarios": self._hardware_scenarios(),
        }

    @staticmethod
    def _hardware_scenarios():
        from diagnostic_mode import inventory, failure_modes, detection_path
        return [dict(board=board, reference=ref, failure=mode,
                     detection=detection_path(ref, board))
                for board, ref in inventory() for mode in failure_modes(ref)]

    def _export_diagnostic_bundle(self):
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "导出诊断包", "crowerfoc_diagnostics.zip", "ZIP (*.zip)")
        if not path:
            return
        from host_tools import create_diagnostic_bundle
        create_diagnostic_bundle(path, self._diagnostic_snapshot())
        self.logmsg(f"诊断包已导出：{path}")

    def _fresh(self):
        return bool(self._connected and self.worker and self.worker.latest and
                    time.monotonic() - self.worker.last_telem < TELEMETRY_TIMEOUT)

    def _ready(self):
        return self._fresh() and not self._reads and all(p in self.param_cache for p in CRITICAL_PARAMS)

    def _idle(self):
        return self._fresh() and self.worker.latest["state"] == 1 and not self._bench_active

    def _can_drive(self):
        return bool(self._ready() and self._idle() and not self._writes and
                    self._pending_state is None and not self.worker.latest["fault"] and
                    self.worker.latest["flags"] & 2 and
                    16 <= self.worker.latest["vbus"] / 1000.0 and
                    0 < self.param_cache[0x0103] <= 40 and
                    self.param_cache[0x0500] in (0, 1, 2))

    def _update_controls(self):
        fresh = self._fresh()
        idle = self._idle()
        editable = self._ready() and idle and not self._writes and self._pending_state is None and not self._restore_pending and not self._save_pending
        self.bt_idle.setEnabled(self._connected)
        fixed_profile = 0x020C in self.param_cache
        cal_valid = 0 < self.param_cache.get(0x0104, 0) <= (20 if fixed_profile else self.param_cache.get(0x0103, 0))
        self.bt_cal.setEnabled(self._can_drive() and cal_valid and not self.rb_legacy.isChecked())
        response_v2=self.param_cache.get(0x062A,0)>=2
        self.bt_bw.setEnabled(self._can_drive() and response_v2 and bool(self.worker.latest.get("flags",0)&4) and
                              cal_valid and not self.rb_legacy.isChecked())
        mapped = self.param_cache.get(0x0100, 0) >= 1
        output_encoder_ok=bool(self.param_cache.get(0x0840,0) and self.param_cache.get(0x0843,0) and
                               math.isfinite(self.param_cache.get(0x0416,float('nan'))))
        output_speed_ok=bool(self.param_cache.get(0x0840,0) and self.param_cache.get(0x0843,0))
        mode_needs_output=self.param_cache.get(0x0500,0) == 2
        bad_enabled_output=bool(self.param_cache.get(0x0840,0) and not self.param_cache.get(0x0843,0))
        self.bt_closed.setEnabled(self._can_drive() and mapped and (not mode_needs_output or output_encoder_ok) and not bad_enabled_output and not self.rb_legacy.isChecked())
        target_enabled = bool(fresh and self.worker.latest["state"] == 3 and
                              not self.worker.latest["fault"] and not self._pending_state and
                              not self._writes and self.param_cache.get(0x0500) in (0, 1, 2) and self.worker.latest.get("command_source",1)!=2)
        self.target_group.setEnabled(target_enabled)
        self.bt_set.setEnabled(target_enabled)
        self.sp_set.setEnabled(target_enabled)
        torque_slider_mode=self.cb_mode.currentData()==0 and self.cb_torque_input.currentData()=="hold"
        self.cb_torque_input.setEnabled(target_enabled and self.cb_mode.currentData()==0)
        self.sl_torque.setEnabled(target_enabled and torque_slider_mode)
        self.ck_encoder2.setEnabled(editable and 0x0840 in self.param_cache)
        self.bt_diagnostics.setEnabled(True)
        for w in (self.cb_mode, self.sp_limit_cur, self.sp_limit_vel,
                  self.bt_limits, self.cb_pid, self.ed_param, self.bt_pset):
            w.setEnabled(editable)
        extended=self.param_cache.get(0x0823,0)>=2
        thermal_editable = editable and extended
        self.ck_fet_temp.setEnabled(thermal_editable and 0x0437 in self.param_cache)
        self.ck_motor_temp.setEnabled(thermal_editable and 0x0432 in self.param_cache)
        self.cb_mode.model().item(1).setEnabled(mapped and not bad_enabled_output)
        self.cb_mode.model().item(2).setEnabled(extended and output_encoder_ok)
        self.sp_limit_vel.setMaximum(230.0 if output_speed_ok else 2100.0)
        self.lb_speed_limit.setText("输出轴速度限制 / rpm" if output_speed_ok else "电机速度限制 / rpm")
        if not output_encoder_ok:
            self.cb_mode.setToolTip("速度模式自动使用电机侧编码器，单位为电机 rpm；位置模式需要健康的输出轴编码器和实测传动比。")
        self.cb_inmode.setEnabled(editable and extended)
        self.bt_save.setEnabled(editable and extended)
        self.bt_backup.setEnabled(editable and extended)
        self.bt_restore.setEnabled(editable and extended)
        self.bt_can.setEnabled(editable and extended and bool(self.worker.latest.get('flags',0)&4) and not self.worker.latest.get('fault',0))
        self.bt_prefresh.setEnabled(fresh)
        self.bt_clear.setEnabled(fresh and idle and bool(self.worker.latest["fault"]))
        self.bt_reboot.setEnabled(fresh and idle and not self._writes)
        self.rb_legacy.setEnabled(fresh and (idle or self._bench_active))
        bench = fresh and self.worker.latest["state"] == 1 and self.rb_legacy.isChecked()
        for w in (self.sl_mod, self.sl_freq, self.bt_bench):
            w.setEnabled(bench and not self.worker.latest["fault"])
        self.cb_port.setEnabled(self.worker is None)
        self.cb_baud.setEnabled(self.worker is None)
        self.btn_refresh.setEnabled(self.worker is None)
        self.music_page.update_device(self.worker.latest if fresh else {},
            bool(self._can_drive() and self.worker.latest.get('flags',0)&4 and self.param_cache.get(0x085F,0)>=1))
        self._paint_state()

    @staticmethod
    def _phase_plot_values(snapshot, raw=False):
        fields = ("ia_raw", "ib_raw", "ic_raw") if raw else ("ia", "ib", "ic")
        values = np.vstack([snapshot[field] for field in fields]).astype(np.float32, copy=False) / 1000.0
        if not raw:
            # Clarke/Park in the firmware rejects zero-sequence current. Remove
            # the same common-mode component here so the visible phase curves
            # represent the vector used by the current controller.
            values = values - np.mean(values, axis=0, keepdims=True)
        return values

    def _paint_state(self):
        fresh = self._fresh()
        latest = self.worker.latest if fresh else {}
        state = latest.get("state")
        fault = latest.get("fault", 0)
        if not fresh:
            text = ("通信失联 · 已请求停止，设备状态无法确认" if self._stale else
                    "正在连接 / 同步 · 等待设备遥测与参数" if self.worker else
                    "未连接 · 操作已禁用，连接后自动同步设备状态与参数")
            color, border = "#f1f3f5", "#adb5bd"
        elif fault:
            text = "设备故障 · 已禁用启动，请检查上方故障原因；排除原因后清除故障，不会自动重启电机"
            color, border = "#fff0f0", "#c9302c"
        elif state == 2:
            kind_name={"sweep":"电流环扫频中"}.get(self._active_cal_kind,"完整标定中")
            text = kind_name+" · 编码器1（电机侧）反馈，操作已锁定，可随时点击停止"
            color, border = "#fff7e6", "#d68a00"
        elif state == 3:
            mode = self.param_cache.get(0x0500, latest.get("mode"))
            text = "闭环运行 · " + ("可发送Iq电流指令（A）" if mode == 0 else "可发送速度或位置指令（rpm / °）") + "；修改参数或模式前请先停止"
            color, border = "#eaf7ee", "#278027"
        elif self._bench_active:
            text = "开环V/f调试 · 电流环未参与调节，停止按钮可退出"
            color, border = "#fff7e6", "#d68a00"
        elif state == 1:
            if not self._ready():
                text = "空闲 · 正在同步设备参数，指令区已禁用"
            elif 0x020C not in self.param_cache and self.param_cache.get(0x0104, 0) > self.param_cache.get(0x0103, 0):
                text = "空闲 · 旧版固件标定电流超过运行限流；请更新固件使用固定5A校准"
            else:
                text = "空闲 · 指令区已变灰。可执行完整标定；换相有效后进入闭环，再发送指令。"
            color, border = "#f1f3f5", "#adb5bd"
        else:
            text = "设备状态不支持直接操作 · 请停止并回到空闲"
            color, border = "#f1f3f5", "#adb5bd"
        self.state_banner.setText(text)
        appearance = (state, bool(fault), fresh, self._bench_active)
        if appearance != self._appearance:
            self._appearance = appearance
            self.state_banner.setStyleSheet(f"padding:10px; background:{color}; border-left:5px solid {border}; color:#333;")
            active = fresh and state in (2, 3, 5) and not fault
            self.pw_dq.setTitle("d/q轴电流" if active else "d/q历史计算值 · 当前未驱动或状态未知", color="#fff" if active else "#888")
            self.pw_dq_live.setTitle("d/q轴电流" if active else "d/q历史计算值 · 当前未驱动或状态未知", color="#fff" if active else "#888")
            self.pw_vec.setTitle("电流矢量（与固件A/B变换一致）")

    # ---------- 操作：停止始终优先，其他操作等待设备回报
    def cmd_state(self, state):
        if state == 1:
            self._torque_slider_released()
        if state==1:
            self._restore_pending=False;self._restore_steps.clear()
        if not self.worker or not self._connected:
            return
        if state == 1:
            self.worker.safe_idle()
            self._writes.clear()
            self._bench_active = False
            self._cal_cancelled = True
            with QtCore.QSignalBlocker(self.rb_legacy):
                self.rb_legacy.setChecked(False)
            self.bench_panel.hide()
            self.sl_mod.setValue(0)
            self.sp_set.setValue(0)
            self._pending_state = (1, time.monotonic() + 3)
            self.logmsg("已请求停止：关闭V/f和i/f、清零目标、切换空闲，等待遥测确认")
        elif state in (2, 3):
            if not self._can_drive() or self.rb_legacy.isChecked():
                self.logmsg("当前无法启动：请确认通信、空闲、无故障、编码器有效且参数已回读")
                return
            if state == 3 and self.param_cache.get(0x0100, 0) < 1:
                self.logmsg("换相参数未知，请先执行完整标定")
                return
            cal_limit = 20 if 0x020C in self.param_cache else self.param_cache[0x0103]
            if state == 2 and not 0 < self.param_cache[0x0104] <= cal_limit:
                self.logmsg("旧固件校准条件不满足；请更新固件使用固定校准配置")
                return
            self.worker.safe_idle()
            self.worker.set_state(state)
            self.sp_set.setValue(0)
            self._pending_state = (state, time.monotonic() + 3)
            if state == 2:
                self._active_cal_kind = "full"
                self._cal_started = time.monotonic()
                self._cal_seen = False
                self._cal_cancelled = False
                self._cal_last_phase = None
                self.param_cache.pop(0x020B, None)
                self.param_cache.pop(0x0208, None)
                self.progress.setRange(0, 100)
                self.progress.setValue(0)
                self.lb_progress.setText("等待完整标定开始…")
                self.lb_cal_hint.setText(f"校准电流 {self.param_cache[0x0104]:g} A；轴需自由转动。可随时点击停止。")
            self.logmsg("已请求完整标定（含R/L与编码器）" if state == 2 else "已请求进入闭环，初始指令为0")
        self._update_controls()

    def cmd_bw_sweep(self):
        if self.param_cache.get(0x062A,0)<2:
            self.logmsg("当前固件不支持扫频协议；请先烧录匹配固件"); return
        if not self.worker or not self._can_drive() or self.rb_legacy.isChecked():
            self.logmsg("扫频需要设备空闲、无故障且使用新版固件"); return
        if not (self.worker.latest.get("flags",0)&4):
            self.logmsg("请先完成完整标定，再执行电流环扫频"); return
        self.worker.safe_idle()
        self.worker.set_param(0x061E,1.0)
        self._active_cal_kind="sweep"
        self._cal_started=time.monotonic(); self._cal_seen=False; self._cal_cancelled=False
        self._cal_last_phase=None; self.param_cache.pop(0x020B,None); self.param_cache[0x0208]=6
        self.progress.setRange(0,100); self.progress.setValue(0)
        self.lb_progress.setText("等待电流环扫频开始…")
        self.lb_cal_hint.setText("d轴扫频100–4000Hz共16点；高频共振/饱和点会标无效，请保持轴自由。")
        self._pending_state=(2,time.monotonic()+3)
        self.logmsg("已请求独立电流环扫频（100–4000Hz，16点）；完整标定参数不会重跑")
        self._update_controls()

    def send_setpoint(self, value):
        if not self.bt_set.isEnabled() or not self._fresh():
            return
        mode = int(self.param_cache.get(0x0500,-1))
        if self.worker.latest.get('command_source')==2:
            self.logmsg('CAN正在控制；请先停止，再由USB重新进入闭环');return
        limit = self.param_cache.get(0x0103 if mode == 0 else 0x0400, 0)
        if mode==0 and self.param_cache.get(0x0106)==1: limit*=self.param_cache.get(0x0105,0)
        if mode==2:
            limit=1e6
            axis_pos=float(self.worker.latest.get('pos_deg',0.0))
            if abs(value-axis_pos)>30.0:
                self.logmsg("位置单次目标限制为当前输出轴角度±30°，未发送")
                return
        if mode not in (0, 1, 2) or not math.isfinite(value) or abs(value) > limit:
            self.logmsg("指令超出设备已确认的限制，未发送")
            return
        if mode==2 and self.param_cache.get(0x0412) and not self.param_cache.get(0x0410,0)<=value<=self.param_cache.get(0x0411,0):
            self.logmsg("指令超出设备已确认的限制，未发送")
            return
        self.worker.set_setpoint(value)
        pid = (0x0505,0x0504,0x0503)[int(mode)]
        self._writes[pid] = (value, time.monotonic() + 3)
        self.worker.get_param(pid)
        self.logmsg(f"发送 {self.lb_setpoint.text()} = {value:g}，等待回读")
        self._update_controls()

    def _torque_input_changed(self, _index):
        show = self.cb_mode.currentData() == 0 and self.cb_torque_input.currentData() == "hold"
        self.slider_box.setVisible(show)
        self.sp_set.setVisible(not show)
        self.bt_set.setVisible(not show)
        self.sl_torque.setValue(0)
        if self._torque_drag:
            self._torque_slider_released()

    def _torque_slider_current(self):
        max_a = max(0.0, float(self.param_cache.get(0x0103, 0.0))) * 0.8
        return self.sl_torque.value() * max_a / 1000.0

    def _torque_slider_command(self):
        amps=self._torque_slider_current()
        return amps*float(self.param_cache.get(0x0105,0.0)) if self.param_cache.get(0x0106)==1 else amps

    def _update_torque_slider_label(self):
        amps=self._torque_slider_current()
        suffix=(f"（{amps*float(self.param_cache.get(0x0105,0.0)):+.3f} Nm）"
                if self.param_cache.get(0x0106)==1 else "")
        self.lb_torque_slider.setText(f"按住并拖动：{amps:+.2f} A{suffix}")

    def _torque_slider_pressed(self):
        if self.cb_mode.currentData() != 0 or not self.bt_set.isEnabled():
            self.sl_torque.setValue(0)
            return
        self._torque_drag = True
        self._send_torque_slider()
        self._torque_timer.start()

    def _send_torque_slider(self):
        if not self._torque_drag or not self.bt_set.isEnabled() or not self._fresh():
            self._torque_slider_released()
            return
        # Drag commands are streamed without per-sample GET/readback requests.
        self.worker.set_setpoint(self._torque_slider_command())

    def _torque_slider_released(self):
        was_dragging = self._torque_drag
        self._torque_drag = False
        self._torque_timer.stop()
        self.sl_torque.setValue(0)
        if was_dragging and self.worker and self._connected and self._fresh():
            self.worker.set_setpoint(0.0)
            self.logmsg("力矩滑块已松开，电流指令归零")

    def _set_encoder2_enabled(self, enabled):
        if not self._ready() or not self._idle() or self._writes:
            with QtCore.QSignalBlocker(self.ck_encoder2):
                self.ck_encoder2.setChecked(bool(self.param_cache.get(0x0840,0)))
            return
        self.do_set(0x0840,1 if enabled else 0)

    def open_diagnostics(self):
        from diagnostic_mode import DiagnosticDialog
        dialog=DiagnosticDialog(self, self.worker.latest if self._fresh() else {},
                                self.param_cache, self._connected, self._fresh())
        dialog.component_results=dict(self._component_results)
        dialog.exec_()
        self._component_results=dict(dialog.component_results)

    def change_mode(self, index):
        self._torque_slider_released()
        if not self._ready() or not self._idle() or self._pending_state or self._writes:
            return
        mode = self.cb_mode.itemData(index)
        if mode not in (0, 1, 2) or (mode==2 and (self.param_cache.get(0x0823,0)<2 or not (self.param_cache.get(0x0840,0) and self.param_cache.get(0x0843,0)))):
            return
        self.worker.safe_idle()
        self.sp_set.setValue(0)
        self.do_set(0x0500, mode)

    def _update_setpoint_range(self):
        mode=int(self.param_cache.get(0x0500,0))
        nm=self.param_cache.get(0x0106)==1
        if mode==2:
            pos=float(self.worker.latest.get('pos_deg',0.0)) if self.worker else 0.0
            lo,hi=(pos-30.0,pos+30.0)
            if self.param_cache.get(0x0412): lo,hi=self.param_cache.get(0x0410,-1e6),self.param_cache.get(0x0411,1e6)
        else:
            limit=self.param_cache.get(0x0103 if mode==0 else 0x0400,1)
            if mode==0 and nm: limit*=self.param_cache.get(0x0105,0)
            lo,hi=-limit,limit
        self.sp_set.setRange(lo,hi)
        motor_speed = mode == 1 and not (self.param_cache.get(0x0840,0) and self.param_cache.get(0x0843,0))
        self.lb_setpoint.setText(('扭矩设定值 / Nm' if nm else 'Iq电流设定值 / A') if mode==0 else (('电机速度设定值 / rpm（编码器1）' if motor_speed else '输出轴速度设定值 / rpm（编码器2）') if mode==1 else '目标输出轴机械角 / °（编码器2，单次≤±30°）'))
        self._sync_input_modes()
        self._torque_input_changed(self.cb_torque_input.currentIndex())

    def _sync_input_modes(self):
        mode=int(self.param_cache.get(0x0500,0));choices=[] if mode==2 else [('直通',1)]
        if self.param_cache.get(0x0823,0)>=2:
            choices+= {0:[('电流/扭矩斜坡',5)],1:[('速度斜坡',2)],2:[('位置滤波',3),('梯形轨迹',4)]}.get(mode,[])
        with QtCore.QSignalBlocker(self.cb_inmode):
            self.cb_inmode.clear()
            for label,value in choices:self.cb_inmode.addItem(label,value)
            index=self.cb_inmode.findData(int(self.param_cache.get(0x0501,1)))
            self.cb_inmode.setCurrentIndex(max(0,index))

    def change_input_mode(self,index):
        if self._ready() and self._idle() and not self._writes:
            value=self.cb_inmode.itemData(index)
            if value is not None:self.do_set(0x0501,value)

    def apply_limits(self):
        if not self._ready() or not self._idle() or self._writes:
            return
        current, velocity = self.sp_limit_cur.value(), self.sp_limit_vel.value()
        group = [(0x0103, current), (0x0400, velocity)]
        try:
            for pid, value in group:
                validate_param(pid, value, current)
        except ValueError as exc:
            self.logmsg(str(exc)); return
        # Write the complete group before requesting readback.
        for pid, value in group:
            self.worker.set_param(pid, value)
            self._writes[pid] = (value, time.monotonic() + 3)
        for pid in [pid for pid, _ in group] + [0x0104, 0x0401]:
            self.worker.get_param(pid)
        self.lb_param_status.setText("限制参数已发送，等待回读确认")
        self._update_controls()

    def do_set(self, pid, value):
        if not self._ready() or not self._idle() or self._writes:
            return
        try:
            validate_param(pid, value, self.param_cache.get(0x0103, 0))
        except ValueError as exc:
            self.logmsg(str(exc)); return
        self.worker.set_param(pid, value)
        self._writes[pid] = (value, time.monotonic() + 3)
        self.worker.get_param(pid)
        if pid in (0x0103, 0x0401):
            self.worker.get_param(0x0401 if pid == 0x0103 else 0x0103)
        self.lb_param_status.setText(f"0x{pid:04X}已发送，等待回读确认")
        self._update_controls()

    def _set_thermal_protection(self, pid, enabled):
        if not self._ready() or not self._idle() or self._writes or pid not in self.param_cache:
            self.logmsg("温度保护开关未发送：请先连接并同步参数，且设备须处于空闲状态")
            old = bool(self.param_cache.get(pid, 0))
            checkbox = self.ck_motor_temp if pid == 0x0432 else self.ck_fet_temp
            with QtCore.QSignalBlocker(checkbox):
                checkbox.setChecked(old)
            return
        if bool(self.param_cache[pid]) == bool(enabled):
            return
        self.do_set(pid, int(bool(enabled)))
        self.lb_thermal_config.setText(
            "温度保护开关已发送，等待设备回读；重启后保留时请点控制页的‘保存’。"
            if pid in self._writes else "温度保护设置未发送，请检查设备状态和日志。")

    def _sync_thermal_checkbox(self, pid, value):
        checkbox = self.ck_motor_temp if pid == 0x0432 else self.ck_fet_temp
        with QtCore.QSignalBlocker(checkbox):
            checkbox.setChecked(bool(value))

    def toggle_bench(self, enabled):
        if not enabled and self.worker and self._connected:
            self.cmd_state(1)
        self._update_controls()

    def apply_bench(self):
        if not self.bt_bench.isEnabled() or not self._fresh():
            return
        self.worker.send_msg(0x02, struct.pack("<H", self.sl_freq.value()))
        self.worker.send_msg(0x01, struct.pack("<H", self.sl_mod.value()))
        self._bench_active = self.sl_mod.value() > 0
        self.lb_mod.setText(f"调制 {self.sl_mod.value()}（已发送）")
        self.logmsg("V/f指令已发送；此模式绕过电流环，停止按钮可关闭")
        self._update_controls()

    def clear_faults(self):
        if self.worker and self._fresh() and self._idle():
            self.worker.clear()
            self.logmsg("已请求清除故障，结果以遥测为准；不会自动启动")

    def reboot(self):
        if self.worker and self._fresh() and self._idle():
            self.worker.safe_idle()
            self.worker.send_msg(MSG_REBOOT)
            self.param_cache.clear()
            self.logmsg("已请求重启；重新连接后检查Flash加载状态和校准有效标志")
            self._update_controls()

    def save_config(self):
        if not self.bt_save.isEnabled():return
        self._save_pending=time.monotonic()+4
        self.worker.send_msg(MSG_SAVE)
        self.logmsg('正在保存，等待设备校验应答…');self._update_controls()

    def on_ack(self,command,status,pid):
        if status:
            reason={1:'参数、格式或功能不支持',2:'当前状态不允许',3:'硬件操作失败'}.get(status,'设备拒绝')
            if pid==0x062A:
                self.param_cache[0x062A]=0.0
                self.logmsg('设备固件未提供扫频扩展能力；扫频已禁用，请烧录匹配固件')
            else:
                self.logmsg(f'设备拒绝0x{command:02X} / 参数0x{pid:04X}：{reason}')
            self._writes.pop(pid,None);self._reads.discard(pid)
            if pid in (0x0432, 0x0437) and self.worker:
                self.worker.get_param(pid)
        if command==MSG_SAVE and self._save_pending:
            self._save_pending=None
            title='配置保存成功' if status==0 else '配置保存失败'
            dialog=QtWidgets.QMessageBox(self)
            dialog.setAttribute(QtCore.Qt.WA_DeleteOnClose);dialog.setWindowTitle(title)
            dialog.setText('配置和有效校准已写入Flash并校验。重启后重新测电流零点。' if status==0 else '设备未确认保存成功，原有有效记录会保留。')
            dialog.show();self._dialogs.append(dialog)
            for p in (0x0820,0x0821,0x0822):self.worker.get_param(p)
        if command==MSG_CONFIG_PATCH and self._restore_pending:
            if status:
                self._restore_pending=False;self._restore_steps.clear();self.worker.send_msg(MSG_CONFIG_PATCH,b'\x03')
            elif self._restore_steps:
                self.worker.send_msg(MSG_CONFIG_PATCH,self._restore_steps.popleft())
                self._restore_deadline=time.monotonic()+4
            else:
                self._restore_pending=False;self.param_cache.clear();self.read_all_params()
                self.logmsg('配置已原子应用到RAM；完整校准后保存到Flash。未自动启动。')
        self._update_controls()

    def backup_config(self):
        if not self.bt_backup.isEnabled():return
        document=configuration_document(self.param_cache,WRITABLE_PARAMS)
        path,_=QtWidgets.QFileDialog.getSaveFileName(self,'导出设备配置','crowerfoc_config.json','JSON (*.json)')
        if path:
            try:Path(path).write_text(json.dumps(document,ensure_ascii=False,indent=2),encoding='utf-8')
            except OSError as exc:self.logmsg(f'导出失败：{exc}');return
            self.logmsg(f'配置已导出：{path}')

    def restore_config(self):
        if not self.bt_restore.isEnabled():return
        path,_=QtWidgets.QFileDialog.getOpenFileName(self,'导入配置（恢复后需重新校准）','','JSON (*.json)')
        if not path:return
        try:
            values=parse_configuration(Path(path).read_text(encoding='utf-8'),WRITABLE_PARAMS)
            for pid,value in values.items():validate_param(pid,value,self.param_cache.get(0x0103,1))
        except (ValueError,OSError) as exc:self.logmsg(f'配置未发送：{exc}');return
        rows=[struct.pack('<Hf',pid,to_wire(pid,value)) for pid,value in sorted(values.items())]
        self._restore_steps=deque([b'\x01'+b''.join(rows[i:i+9]) for i in range(0,len(rows),9)]+[b'\x02'])
        self._restore_pending=True;self._restore_deadline=time.monotonic()+4
        self.worker.safe_idle();self.worker.send_msg(MSG_CONFIG_PATCH,b'\x00')
        self.logmsg('正在校验并分块暂存配置；全部通过后一次性应用。');self._update_controls()

    # ---------- 参数：异步读取、只读项不出现在写入列表
    def read_all_params(self):
        if not self.worker or not self._connected:
            return
        test_only=set(range(0x0220,0x0274))
        ordered = list(CRITICAL_PARAMS) + [pid for pid, *_ in PARAMS if pid not in CRITICAL_PARAMS and pid not in CAL_OPTIONAL_PARAMS and pid not in test_only]
        self._reads = set(ordered)
        for pid in ordered:
            self.worker.get_param(pid)
        for pid in CAL_OPTIONAL_PARAMS:
            if pid not in test_only:
                self.worker.get_param(pid)
        self.lb_param_status.setText("正在异步读取设备参数…")

    def select_param(self):
        pid = self.cb_pid.currentData()
        value = self.param_cache.get(pid)
        self.ed_param.setText("" if value is None else f"{value:.7g}")

    def select_tree_param(self, item, column):
        index = self.cb_pid.findData(item.data(0, QtCore.Qt.UserRole))
        if index >= 0:
            self.cb_pid.setCurrentIndex(index)
            self.select_param()

    def on_param_dblclick(self, item, column):
        pid = item.data(0, QtCore.Qt.UserRole)
        if pid is not None and self.worker and self._fresh():
            if pid not in CAL_OPTIONAL_PARAMS:
                self._reads.add(pid)
            self.worker.get_param(pid)
            self._update_controls()

    def on_param_apply(self):
        pid = self.cb_pid.currentData()
        if pid not in WRITABLE_PARAMS:
            return
        try:
            value = float(self.ed_param.text())
        except ValueError:
            self.logmsg("数值无效，未发送"); return
        self.do_set(pid, value)

    def on_param_reply(self, pid, value):
        self.can_page.on_param(pid, value)
        if not self.worker or (self.sender() is not None and self.sender() is not self.worker):
            return
        if not math.isfinite(value):
            if pid == 0x0416:
                self.param_cache.pop(pid,None)
                self.lb_gear_ratio.setText("实测传动比：未测")
                return
            if pid in (0x0810,0x0811):
                self._reads.discard(pid)
                if pid in self._tree_items:self._tree_items[pid].setText(1,'未接 / 未启用')
                return
            self.logmsg(f"设备返回无效参数 0x{pid:04X}，未用于控制")
            self.param_cache.pop(pid, None)
            return
        self.param_cache[pid] = value
        if pid in (0x0432, 0x0437):
            self._sync_thermal_checkbox(pid, value)
        if pid == 0x0630:
            self._configured_telem_hz = value
        if pid == 0x0840:
            with QtCore.QSignalBlocker(self.ck_encoder2):
                self.ck_encoder2.setChecked(bool(value))
        elif pid == 0x0843:
            self.lb_encoder2_health.setText("正常" if value >= 0.5 else
                ("未就绪 / 读数异常" if self.param_cache.get(0x0840,0) else "已停用"))
        elif pid == 0x0416:
            self.lb_gear_ratio.setText("实测传动比：未测" if not math.isfinite(value) else f"实测传动比：{value:.4f}:1")
        elif pid == 0x0844 and value:
            self.lb_encoder2_health.setToolTip(f"累计读取失败 {int(value)} 次")
        self._reads.discard(pid)
        if not self._reads and not self._writes:
            self.lb_param_status.setText("设备参数已同步")
        item = self._tree_items.get(pid)
        if item is not None:
            item.setText(1, f"0x{int(value):04X}" if pid == 0x0209 else f"{value:.5g}")
            item.setToolTip(1, f"{value:.9g}")
        pending = self._writes.pop(pid, None)
        if pending is not None:
            matched = math.isclose(value, pending[0], rel_tol=1e-5, abs_tol=1e-6)
            text = f"0x{pid:04X}回读确认 = {value:g}" if matched else f"0x{pid:04X}写入未生效：请求{pending[0]:g}，设备{value:g}"
            self.lb_param_status.setText(text)
            self.logmsg(text)
        for key, control in ((0x0103, self.sp_limit_cur), (0x0104, self.sp_cal_cur), (0x0400, self.sp_limit_vel)):
            if pid == key:
                with QtCore.QSignalBlocker(control):
                    control.setValue(value)
        if pid == 0x0500:
            with QtCore.QSignalBlocker(self.cb_mode):
                self.cb_mode.setCurrentIndex(self.cb_mode.findData(int(value)))
            if int(value)==2 and self.worker:self.worker.get_param(0x0501)
        if pid in (0x0103,0x0400,0x0500,0x0501,0x0105,0x0106,0x0410,0x0411,0x0412,0x0823,0x0416):
            self._update_setpoint_range()
        if pid in self._result_wait:
            self._result_wait.remove(pid)
            self._result_data[pid] = value
            if not self._result_wait:
                self.show_bode_result() if self._active_cal_kind=="sweep" else self.show_result(True)
        if self._ready() and not self._writes:
            self._update_controls()

    def show_result(self, success, reason=""):
        if self._closing:
            return
        dialog = QtWidgets.QDialog(self)
        dialog.setAttribute(QtCore.Qt.WA_DeleteOnClose)
        dialog.setWindowTitle("完整标定结果" if success else "完整标定未完成")
        layout = QtWidgets.QVBoxLayout(dialog)
        headline = QtWidgets.QLabel("电机与换相参数标定完成（齿槽补偿关闭）" if success else reason)
        headline.setWordWrap(True)
        layout.addWidget(headline)
        if success:
            form = QtWidgets.QFormLayout()
            for pid, label, unit in ((0x0100, "极对数", ""), (0x0201, "编码器1方向", ""),
                                      (0x0200, "编码器1电角度偏置", "°"), (0x0101, "相电阻估算", "mΩ"),
                                      (0x0102, "相电感估算", "µH"), (0x0207, "角度圆周一致性", "0~1"),
                                      (0x0107, "电流环带宽目标", "Hz"),
                                      (0x0215, "齿槽补偿表有效", "0/1"),
                                      (0x0303, "电流环 Kp 回读", "V/A"),
                                      (0x0304, "电流环 Ki 回读", "V/(A·s)")):
                value = self._result_data.get(pid)
                if pid == 0x0220 and value is not None:
                    shown = "<250 Hz" if value < 1 else ("≥2000 Hz" if value > 2000 else f"{value:.6g} Hz")
                    form.addRow(label, QtWidgets.QLabel(shown))
                    continue
                form.addRow(label, QtWidgets.QLabel("未获取" if value is None else f"{value:.7g} {unit}"))
            form.addRow("扫频",QtWidgets.QLabel("独立操作：点击“电流环扫频 / 波特图”"))
            enc2_states={0:"未启用",1:"测量中",2:"通信异常",3:"角度无效",4:"运动映射未完成",5:"标定完成"}
            form.addRow("ENC2 标定",QtWidgets.QLabel(enc2_states.get(int(self._result_data.get(0x084B,-1)),"固件未提供")))
            for pid,label in ((0x0416,"实测电机角/输出角"),(0x084C,"双编码器映射 R²"),(0x0841,"ENC2方向"),(0x0842,"ENC2机械零位 / °")):
                value=self._result_data.get(pid)
                form.addRow(label,QtWidgets.QLabel(f"{value:.6g}" if value is not None and math.isfinite(value) else "未完成"))
            layout.addLayout(form)
            note = QtWidgets.QLabel("R为两档电流斜率估算，L为脉冲估算。扫频已拆分为独立操作；请在完整标定成功后另行启动。齿槽补偿已关闭。\n"
                                   "如需重启后保留，请在结果确认后执行“保存参数”。\n"
                                   "设备当前应为空闲；闭环需另行启动，初始指令为0。")
            note.setWordWrap(True)
            layout.addWidget(note)
            if reason:
                layout.addWidget(QtWidgets.QLabel(reason))
        buttons = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Ok)
        buttons.button(QtWidgets.QDialogButtonBox.Ok).setText("确定")
        buttons.accepted.connect(dialog.accept)
        layout.addWidget(buttons)
        self._dialogs.append(dialog)
        dialog.destroyed.connect(lambda _=None, dialogs=self._dialogs, item=dialog:
                                 dialogs.remove(item) if item in dialogs else None)
        dialog.show()
        self._result_wait.clear()
        if self._active_cal_kind == "full":
            self._active_cal_kind = None

    def show_bode_result(self):
        if self._closing:
            return
        dialog = QtWidgets.QDialog(self)
        dialog.setAttribute(QtCore.Qt.WA_DeleteOnClose)
        dialog.setWindowTitle("电流环扫频 · 波特图")
        screen = dialog.screen().availableGeometry()
        dialog.setMinimumSize(min(760, screen.width()), min(520, screen.height()))
        dialog.resize(min(1250, int(screen.width()*.90)), min(820, int(screen.height()*.90)))
        outer = QtWidgets.QVBoxLayout(dialog)
        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
        content = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(content)
        scroll.setWidget(content)
        outer.addWidget(scroll, 1)
        freq = [100,150,200,300,400,500,650,800,1000,1250,1500,1800,2200,2700,3300,4000]
        gain = [self._result_data.get(pid, float("nan")) for pid in range(0x0224, 0x0234)]
        lag_raw = [self._result_data.get(pid, float("nan")) for pid in range(0x0234, 0x0244)]
        # Firmware reports lag on [0, 360); keep the negative phase unwrapped
        # so crossing -180 degrees does not create a false upward jump.
        lag = [-float(v) if math.isfinite(float(v)) else float("nan") for v in lag_raw]
        amplitude = [20 * math.log10(max(float(v), 1e-9)) if math.isfinite(float(v)) else float("nan") for v in gain]
        command = [self._result_data.get(pid, float("nan")) for pid in range(0x0244, 0x0254)]
        saturation = [self._result_data.get(pid, float("nan")) for pid in range(0x0254, 0x0264)]
        valid = [self._result_data.get(pid, 0.0) >= 0.5 for pid in range(0x0264, 0x0274)]
        x = np.log10(np.asarray(freq, dtype=float))
        ticks = [[(float(v), str(f)) for v, f in zip(x, freq)]]
        amp = pg.PlotWidget(title="电流闭环幅频响应")
        amp.setMinimumHeight(180); amp.setMaximumHeight(290)
        amp.setLabel("left", "幅值 / dB")
        amp.setLabel("bottom", "频率 / Hz（对数）")
        amp.showGrid(x=True, y=True, alpha=.35)
        amp.getAxis("bottom").setTicks(ticks)
        def finite_series(values, predicate):
            pairs=[(float(x[i]),float(v)) for i,v in enumerate(values)
                   if predicate(i) and math.isfinite(float(v))]
            return ([p[0] for p in pairs],[p[1] for p in pairs])
        amp_valid=finite_series(amplitude,lambda i:valid[i])
        amp_invalid=finite_series(amplitude,lambda i:not valid[i])
        if amp_valid[0]:
            amp.plot(x, [v if valid[i] else float("nan") for i,v in enumerate(amplitude)],
                     pen=pg.mkPen("#55ddff", width=2), connect="finite")
            amp.plot(*amp_valid, pen=None, symbol="o", symbolSize=8,
                     symbolBrush="#55ddff", name="有效点")
        if amp_invalid[0]:
            amp.plot(*amp_invalid, pen=None, symbol="x", symbolSize=11,
                     symbolPen=pg.mkPen("#ff9966", width=2), name="无效点（仍显示）")
        phase = pg.PlotWidget(title="电流闭环相频响应")
        phase.setMinimumHeight(180); phase.setMaximumHeight(290)
        phase.setLabel("left", "闭环相位 / °（滞后为负）")
        phase.setLabel("bottom", "频率 / Hz（对数）")
        phase.showGrid(x=True, y=True, alpha=.35)
        phase.getAxis("bottom").setTicks(ticks)
        phase_valid=finite_series(lag,lambda i:valid[i])
        phase_invalid=finite_series(lag,lambda i:not valid[i])
        if phase_valid[0]:
            phase.plot(x, [v if valid[i] else float("nan") for i,v in enumerate(lag)],
                       pen=pg.mkPen("#ffbb55", width=2), connect="finite")
            phase.plot(*phase_valid, pen=None, symbol="o", symbolSize=8,
                       symbolBrush="#ffbb55")
        if phase_invalid[0]:
            phase.plot(*phase_invalid, pen=None, symbol="x", symbolSize=11,
                       symbolPen=pg.mkPen("#ff9966", width=2))
        layout.addWidget(amp, 1)
        layout.addWidget(phase, 1)
        cutoff = self._result_data.get(0x0220)
        target = self._result_data.get(0x0223)
        status = "通过" if target is not None and target >= .5 else "未达到"
        cutoff_text=(f"{cutoff:.4g} Hz" if cutoff is not None and cutoff>=1 else "未能确定（有效低频基准或连续有效点不足）")
        summary = f"实测 -3dB 带宽：{cutoff_text} · 1kHz目标：{status}" if cutoff is not None else "扫频数据不完整"
        summary += " · 全部实测点均保留；橙色×表示该点不参与有效带宽计算。"
        summary_label=QtWidgets.QLabel(summary); summary_label.setWordWrap(True)
        layout.addWidget(summary_label)
        table=QtWidgets.QTableWidget(len(freq),6)
        table.setHorizontalHeaderLabels(("频率 / Hz","增益 / dB","相位滞后 / °","注入电流 / A","饱和 / %","判定"))
        table.verticalHeader().setVisible(False)
        table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        table.setSelectionMode(QtWidgets.QAbstractItemView.NoSelection)
        table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        table.setMaximumHeight(255)
        for row,f in enumerate(freq):
            values=(f, amplitude[row], lag[row], command[row], saturation[row], "有效" if valid[row] else "无效")
            for col,value in enumerate(values):
                text=("--" if isinstance(value,float) and not math.isfinite(value) else
                      f"{value:.3g}" if isinstance(value,float) else str(value))
                item=QtWidgets.QTableWidgetItem(text)
                if col==5 and not valid[row]: item.setForeground(QtGui.QBrush(QtGui.QColor("#cc6633")))
                table.setItem(row,col,item)
        layout.addWidget(table)
        buttons = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Ok)
        buttons.button(QtWidgets.QDialogButtonBox.Ok).setText("确定")
        buttons.accepted.connect(dialog.accept)
        outer.addWidget(buttons)
        self._dialogs.append(dialog)
        dialog.destroyed.connect(lambda _=None, dialogs=self._dialogs, item=dialog:
                                 dialogs.remove(item) if item in dialogs else None)
        dialog.show()
        self._result_wait.clear()
        self._active_cal_kind = None

    def _update_control_readouts(self):
        fresh = self._fresh()
        if not fresh:
            values = {
                "device": "未连接", "axis": "--", "fault": "--", "bus": "--",
                "phase": "--", "dq": "--", "angle": "--", "speed": "--",
                "position": "--", "rate": "--", "enc2_angle":"--", "enc2_speed":"--",
                "enc2_position":"--", "ratio":"--",
            }
            self.lb_readout_note.setText("连接设备后自动显示遥测与状态。控制项在连接并读回关键参数前保持禁用。")
        else:
            latest = self.worker.latest
            state = int(latest["state"])
            fault = int(latest["fault"])
            state_name = AXIS_STATES[state] if 0 <= state < len(AXIS_STATES) else "未知"
            fault_text = "无" if not fault else f"异常 0x{fault:04X}"
            values = {
                "device": f"已连接 · {self.worker.port}",
                "axis": state_name,
                "fault": fault_text,
                "bus": f"{latest['vbus'] / 1000.0:.2f} V",
                "phase": " / ".join(f"{latest[k] / 1000.0:.2f}" for k in ("ia", "ib", "ic")) + " A",
                "dq": f"{latest['id'] / 1000.0:.2f} / {latest['iq'] / 1000.0:.2f} A",
                "angle": f"{latest['ang']:.2f}° / {latest['ele']:.2f}°",
                "speed": f"{latest.get('enc1_vel',float('nan')):.2f} rpm",
                "position": f"{latest.get('enc1_pos',float('nan')):.2f}°",
                "enc2_angle": f"{latest.get('enc2_ang',float('nan')):.2f}°",
                "enc2_speed": f"{latest.get('enc2_vel',float('nan')):.2f} rpm",
                "enc2_position": f"{latest.get('enc2_pos',float('nan')):.2f}°",
                "ratio": f"{latest.get('encoder_ratio',float('nan')):.5g}",
                "rate": self.lb_rate.text().replace("速率", "").strip(),
            }
            self.lb_readout_note.setText("状态和读数来自最近一帧设备遥测；曲线保留最近采样窗口。")
        for key, value in values.items():
            self.readout_labels[key].setText(value.replace("nan","--"))
        self.readout_labels["fault"].setStyleSheet(
            "font-size:15pt; font-weight:600; color:#c9302c" if fresh and self.worker.latest["fault"]
            else "font-size:15pt; font-weight:600; color:#278027" if fresh
            else "font-size:15pt; font-weight:600; color:#687386")

    def _set_live_plot_state(self, connected):
        connected = bool(connected)
        if connected == self._live_plot_connected:
            return
        self._live_plot_connected = connected
        if connected:
            self.lb_current_view.setText("实时遥测 · 电流环在设备端以25kHz采样")
            titles = ("三相电流", "d/q 轴电流", "ENC1机械角 / 电角度 · ENC2机械角")
        else:
            self.lb_current_view.setText("未连接设备 · 连接后显示三相电流、d/q 电流与编码器1机械角/电角度")
            titles = ("三相电流 · 等待连接", "d/q 电流 · 等待连接", "编码器1机械角 / 电角度 · 等待连接")
        for plot, title in zip((self.pw_cur_live, self.pw_dq_live, self.pw_enc_live), titles):
            plot.setTitle(title)
        if not connected:
            # Telemetry buffers belong to the previous connection. Leaving them
            # visible beneath a “waiting for connection” title looks like live noise.
            for curves in (self.c_cur, self.c_dq, self.c_mot, self.c_vel,
                           self.c_pos, self.c_mot2, self.c_vel2, self.c_pos2, self.c_duty, self.c_cur_live,
                           self.c_dq_live, self.c_enc_live):
                for curve in curves.values():
                    curve.setData([], [])
            self.v_traj.setData([], [])
            self.v_now.setData([], [])

    def _update_device(self):
        self._update_control_readouts()
        now = time.monotonic()
        if self._connected and self.worker and self.worker.latest and not self._fresh():
            if not self._stale:
                self._stale = True
                self.worker.safe_idle()
                self._pending_state = None
                self._writes.clear()
                self.logmsg("遥测超时，已请求停止；设备状态待确认")
                if self._cal_seen:
                    self.show_result(False, "通信超时，已请求停止，标定结果无法确认")
                    self._cal_seen = False
                    self._cal_started = None
            self.lb_conn.setText("遥测超时 · 已请求停止")
        elif self._fresh():
            self._stale = False
            self.lb_conn.setText(f"已连接 {self.worker.port}")
        for pid, (_, deadline) in list(self._writes.items()):
            if now > deadline:
                del self._writes[pid]
                self.lb_param_status.setText(f"0x{pid:04X}回读超时，写入结果未知")
                self.logmsg(self.lb_param_status.text())
                self.param_cache.pop(pid, None)
                if pid in (0x0432, 0x0437) and self.worker:
                    self.worker.get_param(pid)
        if self._pending_state and self._fresh():
            requested, deadline = self._pending_state
            current = self.worker.latest["state"]
            if current == requested:
                self.logmsg(f"设备确认：{AXIS_STATES[current]}")
                self._pending_state = None
            elif self.worker.latest["fault"] or now > deadline:
                self._pending_state = None
                self.logmsg("状态请求未被设备接受，已请求回到空闲")
                self.worker.safe_idle()
                if requested == 2:
                    self.show_result(False, "设备未进入完整标定，请检查故障、编码器及母线电压")
                    self._cal_started = None
        if self._fresh():
            latest = self.worker.latest
            state = latest["state"]
            fault = latest["fault"]
            name = AXIS_STATES[state] if state < len(AXIS_STATES) else "未知"
            self.lb_state.setText(f"状态 {name}" + (" / V/f调试" if self._bench_active else ""))
            self.lb_vbus.setText(f"母线 {latest['vbus']/1000:.2f}V")
            if 'fet_temp_C' in latest:
                motor=f"{latest['motor_temp_C']:.1f}℃" if math.isfinite(latest.get('motor_temp_C',float('nan'))) else '未接'
                owner={0:'空闲',1:'USB',2:'CAN'}.get(int(latest.get('command_source',0)),'未知')
                fet=f"{latest['fet_temp_C']:.1f}℃" if math.isfinite(latest['fet_temp_C']) else '未接 / 未启用'
                self.lb_thermal.setText(f"板载 {fet} · 绕组 {motor} · 控制 {owner}")
            fault_text = "、".join(label for bit, label in enumerate(FAULT_NAMES) if fault & (1 << bit))
            self.lb_fault.setText(f"故障 {fault_text or '无'}")
            self.lb_fault.setToolTip(f"0x{fault:04X}")
            self.lb_fault.setStyleSheet("color:#c9302c" if fault else "color:#278027")
            motor_status="编码器1（电机侧）正常" if latest['flags'] & 2 else "编码器1（电机侧）异常"
            out_enabled=bool(self.param_cache.get(0x0840,0))
            if "position" in self.readout_title_labels:
                self.readout_title_labels["position"].setText("编码器2输出轴机械角" if out_enabled else "编码器1电机机械角")
            out_ok=bool(self.param_cache.get(0x0843,0))
            out_status=("编码器2（输出轴）正常" if out_ok else "编码器2（输出轴）异常") if out_enabled else "编码器2（输出轴）未启用"
            self.lb_encoder.setText(motor_status+" · "+out_status)
            mapped = self.param_cache.get(0x0100, 0) > 0
            self.lb_mapping.setText("换相已标定" if latest['flags'] & 4 else ("换相手动配置" if mapped else "换相未配置"))
            if fault and fault != self._last_fault:
                self.logmsg(f"设备故障 0x{fault:04X}：{fault_text}")
            self._last_fault = fault
            if state == 2:
                self._cal_seen = True
                if now >= self._diag_due:
                    self.worker.get_param(0x0208)
                    self.worker.get_param(0x020B)
                    self._diag_due = now + 0.35
                phase = int(self.param_cache.get(0x0208, 0))
                kind = self._active_cal_kind or "full"
                prefix={"sweep":"电流环扫频 · "}.get(kind,"完整标定 · ")
                self.lb_progress.setText(prefix+("频率响应测量" if kind=="sweep" else CAL_STAGES.get(phase,"处理中")))
                hints = CAL_HINTS if 0x020C in self.param_cache else CAL_HINTS_LEGACY
                self.lb_cal_hint.setText(hints.get(phase, "正在等待设备阶段反馈，可随时停止。"))
                if phase != self._cal_last_phase:
                    self.logmsg(hints.get(phase, f"校准阶段 {phase}"))
                    self._cal_last_phase = phase
                if 0x020B in self.param_cache:
                    self.progress.setRange(0, 100)
                    self.progress.setValue(min(99, max(0, int(self.param_cache[0x020B]))))
                elif self._cal_started is None or 0x020C in self.param_cache:
                    self.progress.setRange(0, 0)
                else:
                    self.progress.setRange(0, 100)
                    self.progress.setValue(min(99, int((now-self._cal_started)/27.1*100)))
            elif self._cal_seen:
                self._cal_seen = False
                self._cal_started = None
                self.progress.setRange(0, 100)
                if state == 1 and latest['flags'] & 4 and not fault and not self._cal_cancelled:
                    self.progress.setValue(100)
                    is_sweep=self._active_cal_kind=="sweep"
                    self.lb_progress.setText("扫频完成 · 正在读取波特图数据" if is_sweep else "完整标定完成 · 正在读取结果")
                    self.lb_cal_hint.setText("读取16个频点的幅值、相位和有效性。" if is_sweep else "完整标定结束；频响测试为独立操作。")
                    self._result_wait = ({0x0220,0x0221,0x0222,0x0223} | set(range(0x0224,0x0274))) if is_sweep else {
                        0x0100,0x0101,0x0102,0x0107,0x0200,0x0201,0x0207,0x0215,0x0303,0x0304,
                        0x0416,0x0841,0x0842,0x084B,0x084C}
                    self._result_data.clear()
                    self._result_deadline = now + 4
                    for pid in self._result_wait:
                        self.worker.get_param(pid)
                    if not is_sweep:
                        self.read_all_params()
                else:
                    self.lb_progress.setText("标定已中止" if self._cal_cancelled else "标定失败")
                    self.lb_cal_hint.setText("已退出校准，保持空闲。" + ("用户请求停止。" if self._cal_cancelled else "请查看故障及最后阶段日志。"))
                    self.show_result(False, "用户停止了标定" if self._cal_cancelled else f"标定未完成：{fault_text or '设备已退出标定'}")
            self._last_state = state
        if self._result_wait and now > self._result_deadline:
            self.logmsg("标定完成，但部分结果读取超时")
            self.show_result(True, "部分结果未获取")
        self._update_controls()

    # ---------- 绘图
    def tick(self):
        now=time.monotonic()
        if self._save_pending and now>self._save_pending:
            self._save_pending=None;self.logmsg('保存应答超时，结果未确认');self._update_controls()
        if self._restore_pending and now>self._restore_deadline:
            self._restore_pending=False;self._restore_steps.clear()
            if self.worker:self.worker.send_msg(MSG_CONFIG_PATCH,b'\x03')
            self.logmsg('导入超时，取消暂存；请回读配置确认');self._update_controls()
        self._update_device()
        self._set_live_plot_state(self._fresh())
        if not self.worker:
            return
        if self._fresh() and now >= self._encoder2_poll_at:
            self._encoder2_poll_at=now+0.75
            for pid in (0x0840,0x0843,0x0844,0x0416):self.worker.get_param(pid)
        r = self.worker
        if r.count == 0:
            return
        s = r.snapshot(self.window)
        x = s["time_s"] - s["time_s"][0] if s["time_s"].size else s["time_s"]
        raw_view = self.cb_current_view.currentIndex() == 1
        raw_available = np.any(np.isfinite(s["ia_raw"]))
        display_note = "按设备时间顺序连线；环路采样率25kHz。"
        self.lb_current_view.setText("原始采样 / A · 未剔除共模与尖峰；" + display_note + "上位机显示设备遥测" if raw_view and raw_available else
                                     "电流环输入 / A · 已滤波并剔除三相共模；" + display_note + "原始过流保护独立运行" if raw_available else
                                     "设备未提供原始采样帧；电流环输入 / A · " + display_note)
        phase_values = self._phase_plot_values(s, raw=raw_view and raw_available)
        for i, f in enumerate(("ia", "ib", "ic")):
            self.c_cur[f].setData(x, phase_values[i])
            self.c_cur_live[f].setData(x, phase_values[i])
        for f, cu in self.c_dq.items():
            # Firmware retains the last computed d/q while gates are off.
            # Break idle segments instead of drawing old current as live data.
            values = np.where(s["state"] == 1, np.nan, s[f]) / 1000.0
            cu.setData(x, values) if np.any(np.isfinite(values)) else cu.setData([], [])
        for f, cu in self.c_dq_live.items():
            values = np.where(s["state"] == 1, np.nan, s[f]) / 1000.0
            cu.setData(x, values) if np.any(np.isfinite(values)) else cu.setData([], [])
        for f, cu in self.c_mot.items():
            cu.setData(x, s[f])
        for f, cu in self.c_enc_live.items():
            cu.setData(x, s[f])
        for fields in (self.c_vel, self.c_pos,self.c_mot2,self.c_vel2,self.c_pos2):
            for f, cu in fields.items():
                cu.setData(x, s[f])
        for f, cu in self.c_duty.items():
            cu.setData(x, s[f] / PWM_ARR * 100.0)

        m = s["ia"].size
        seg = min(m, 400)
        ia, ib, ic = (s[f][m - seg:] for f in ("ia", "ib", "ic"))
        alpha = (2.0 * ia - ib - ic) / 3.0
        beta = (ib - ic) / np.sqrt(3.0)  # match firmware's three-phase Clarke
        self.v_traj.setData(alpha, beta)
        if alpha.size:
            self.v_now.setData([alpha[-1]], [beta[-1]])

    def closeEvent(self, ev):
        self._torque_slider_released()
        if self.worker:
            self._closing = True
            self._connected = False
            self.worker.stop()
            ev.ignore()
            return
        ev.accept()


def main():
    QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling, True)
    QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_UseHighDpiPixmaps, True)
    app = QtWidgets.QApplication(sys.argv)
    try:
        pt = int(os.environ.get("VIEWER_FONT_PT", "11"))
    except ValueError:
        pt = 11
    f = app.font()
    f.setFamily("Microsoft YaHei UI")
    f.setPointSize(pt)
    app.setFont(f)
    w = MainWindow()
    w.showMaximized()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
