"""Fault detail and offline CAN utilities for the CrowerFOC UI."""
import json
import math
import time
import zipfile
from collections import deque

from PyQt5 import QtWidgets

from crowerfoc_protocol import FAULT_NAMES
from mit_can import (pack_control, unpack_control, special_command, unpack_feedback,
                     CMD_ENABLE, CMD_DISABLE, CMD_ZERO)


FAULT_GUIDANCE = {
    0: "母线过压：检查电源设定、回馈能量和母线采样分压。",
    1: "母线欠压：检查电源限流、线束压降、连接器和母线采样。",
    2: "过流：检查触发时的故障遥测、电流零点、相线、采样电阻和驱动时序。",
    3: "功率级过温：检查功率器件、散热器和风道；当前板载NTC未焊时温度不可信。",
    4: "电机过温：检查绕组NTC接线和参数；未接传感器时不能据此判断电机温度。",
    5: "编码器异常：检查供电、SSC时钟/数据/片选、CRC/STAT和角度连续性。",
    6: "换相未配置/标定不一致：回读极对数、方向和偏置，重新执行完整标定。",
    7: "栅极驱动异常：检查驱动器供电、使能、故障输出、栅极电阻和相桥波形。",
    8: "ADC采样异常：检查PWM同步触发、ADC超时计数、采样标志和电流放大器输出。",
    9: "主机心跳超时：检查USB/CAN连接和主控周期，重新使能前先确认电流指令为零。",
    10: "校准失败：查看失败阶段、目标/实测电流和R/L拟合诊断，检查电机与相线。",
    12: "位置越界：检查输出轴编码器方向/零点、机械行程和位置限位参数。",
    13: "控制周期超时：检查ISR周期诊断、遥测负载和临界区耗时。",
    14: "急停：检查急停输入及外部停机链路。",
    15: "配置异常：导出当前参数，检查范围/校验状态后再恢复有效配置。",
}


def create_diagnostic_bundle(path, snapshot):
    """Create a diagnostic ZIP containing configuration, status, fault history and logs."""
    def json_default(value):
        if hasattr(value, "item"):
            return value.item()
        return str(value)
    with zipfile.ZipFile(path, "w", compression=zipfile.ZIP_DEFLATED) as zf:
        zf.writestr("诊断信息.json", json.dumps(snapshot.get("metadata", {}),
                                                   ensure_ascii=False, indent=2, default=json_default))
        zf.writestr("设备状态.json", json.dumps(snapshot.get("status", {}),
                                                     ensure_ascii=False, indent=2, default=json_default))
        zf.writestr("设备参数.json", json.dumps(snapshot.get("parameters", {}),
                                                      ensure_ascii=False, indent=2, default=json_default))
        zf.writestr("器件排查记录.json", json.dumps(snapshot.get("component_checklist", {}),
                                                         ensure_ascii=False, indent=2, default=json_default))
        zf.writestr("故障历史.json", json.dumps(snapshot.get("fault_history", []),
                                                   ensure_ascii=False, indent=2, default=json_default))
        zf.writestr("网表器件与失效情景.json", json.dumps(snapshot.get("hardware_scenarios", []),
                                                             ensure_ascii=False, indent=2, default=json_default))
        zf.writestr("上位机日志.txt", snapshot.get("logs", ""))


class FaultDetailWidget(QtWidgets.QWidget):
    def __init__(self, bundle_callback=None, parent=None):
        super().__init__(parent)
        self.bundle_callback = bundle_callback
        self.history = deque(maxlen=200)
        self.last_fault = None
        layout = QtWidgets.QVBoxLayout(self)
        self.active = QtWidgets.QLabel("当前故障：等待设备状态")
        self.active.setStyleSheet("font-size:18px;font-weight:bold")
        layout.addWidget(self.active)
        body = QtWidgets.QHBoxLayout()
        self.history_list = QtWidgets.QListWidget()
        self.detail = QtWidgets.QTextBrowser()
        body.addWidget(self.history_list, 1)
        body.addWidget(self.detail, 2)
        layout.addLayout(body, 1)
        row = QtWidgets.QHBoxLayout()
        self.export_button = QtWidgets.QPushButton("导出诊断包…")
        self.clear_button = QtWidgets.QPushButton("清除本地历史")
        row.addWidget(self.export_button)
        row.addWidget(self.clear_button)
        row.addStretch()
        layout.addLayout(row)
        self.history_list.currentRowChanged.connect(self._show_selected)
        self.clear_button.clicked.connect(self._clear)
        self.export_button.clicked.connect(lambda: self.bundle_callback() if self.bundle_callback else None)

    def on_sample(self, sample):
        fault = int(sample.get("fault", 0) or 0)
        names = [f"{bit}: {name}" for bit, name in enumerate(FAULT_NAMES) if fault & (1 << bit)]
        self.active.setText("当前故障：" + ("、".join(names) if names else "无故障"))
        self.active.setStyleSheet("font-size:18px;font-weight:bold;color:" + ("#c9302c" if fault else "#278027"))
        if fault != self.last_fault and fault:
            event = {"time": time.strftime("%Y-%m-%d %H:%M:%S"), "fault": fault,
                     "names": names, "sample": dict(sample)}
            self.history.append(event)
            self.history_list.addItem(f"{event['time']} · {'、'.join(names)} · 0x{fault:04X}")
            self.history_list.setCurrentRow(self.history_list.count() - 1)
        self.last_fault = fault

    def _show_selected(self, row):
        if row < 0 or row >= len(self.history):
            return
        event = list(self.history)[row]
        lines = [f"时间：{event['time']}", f"故障位：0x{event['fault']:04X}", "", "建议检查："]
        for bit in range(len(FAULT_NAMES)):
            if event["fault"] & (1 << bit):
                lines.append("• " + FAULT_GUIDANCE.get(bit, "导出诊断包并检查对应硬件状态。"))
        lines += ["", "触发时遥测：", json.dumps(event["sample"], ensure_ascii=False, indent=2, default=str)]
        self.detail.setPlainText("\n".join(lines))

    def _clear(self):
        self.history.clear()
        self.history_list.clear()
        self.detail.clear()


class CanDebugWidget(QtWidgets.QWidget):
    """Offline MIT-layout frame tool plus read-only MCU CAN diagnostics."""
    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QtWidgets.QVBoxLayout(self)
        note = QtWidgets.QLabel("MIT兼容8字节布局调试器（本项目字段为电流语义）。可离线打包/解码；USB当前不提供CAN报文转发，实际收发需CAN适配器或主控。")
        note.setWordWrap(True)
        layout.addWidget(note)
        self.tabs = QtWidgets.QTabWidget()
        layout.addWidget(self.tabs, 1)
        build = QtWidgets.QWidget()
        form = QtWidgets.QFormLayout(build)
        self.node = QtWidgets.QSpinBox(); self.node.setRange(1, 7); self.node.setValue(1)
        self.pos = self._spin(-12.5 * 180.0 / math.pi, 12.5 * 180.0 / math.pi, 0.0)
        self.vel = self._spin(-30 * 60.0 / (2.0 * math.pi), 30 * 60.0 / (2.0 * math.pi), 0)
        self.kp = self._spin(0, 500, 0)
        self.kd = self._spin(0, 5, 0)
        self.iq = self._spin(-40, 40, 0)
        for label, widget in (("标准帧 ID", self.node), ("位置 / °", self.pos),
                              ("速度 / rpm", self.vel), ("位置误差电流增益 / A/rad", self.kp),
                              ("Kd / A/(rad/s)", self.kd), ("Iq前馈 / A", self.iq)):
            form.addRow(label, widget)
        self.frame = QtWidgets.QLineEdit(); self.frame.setReadOnly(True)
        self.make = QtWidgets.QPushButton("生成控制帧")
        self.decode = QtWidgets.QPushButton("解码下方帧")
        self.special = QtWidgets.QComboBox()
        for label, opcode in (("使能 FC", CMD_ENABLE), ("停止 FD", CMD_DISABLE), ("置零 FE", CMD_ZERO)):
            self.special.addItem(label, opcode)
        self.make_special = QtWidgets.QPushButton("生成特殊帧")
        form.addRow(self.make, self.frame)
        form.addRow(self.special, self.make_special)
        self.decode_text = QtWidgets.QLineEdit(); self.decode_text.setPlaceholderText("输入16位十六进制数据，例如 80 00 80 00 00 00 08 00")
        self.decoded = QtWidgets.QLabel("尚未解码"); self.decoded.setWordWrap(True)
        form.addRow(self.decode_text, self.decode)
        form.addRow("解码结果", self.decoded)
        self.make.clicked.connect(self.make_control)
        self.make_special.clicked.connect(self.make_special_frame)
        self.decode.clicked.connect(self.decode_frame)
        self.tabs.addTab(build, "帧工具")
        diag = QtWidgets.QWidget()
        dv = QtWidgets.QVBoxLayout(diag)
        self.diag_table = QtWidgets.QTableWidget(0, 2)
        self.diag_table.setHorizontalHeaderLabels(["设备CAN诊断项", "数值"])
        self.diag_table.horizontalHeader().setStretchLastSection(True)
        dv.addWidget(self.diag_table)
        self.refresh = QtWidgets.QPushButton("通过USB读取设备CAN状态")
        dv.addWidget(self.refresh)
        self.refresh.clicked.connect(self.refresh_can_stats)
        self.tabs.addTab(diag, "设备状态")
        self.can_worker = None

    @staticmethod
    def _spin(lo, hi, value):
        w = QtWidgets.QDoubleSpinBox(); w.setRange(lo, hi); w.setDecimals(4); w.setValue(value)
        return w

    def set_worker(self, worker):
        self.can_worker = worker

    def make_control(self):
        position_rad = math.radians(self.pos.value())
        velocity_rad_s = self.vel.value() * (2.0 * math.pi / 60.0)
        data = pack_control(position_rad, velocity_rad_s, self.kp.value(),
                            self.kd.value(), self.iq.value())
        self.frame.setText(f"ID=0x{self.node.value():03X} DLC=8 DATA=" + data.hex(" ").upper())

    def make_special_frame(self):
        data = special_command(self.special.currentData())
        self.frame.setText(f"ID=0x{self.node.value():03X} DLC=8 DATA=" + data.hex(" ").upper())

    def decode_frame(self):
        try:
            data = bytes.fromhex(self.decode_text.text().replace(",", " "))
            if len(data) != 8:
                raise ValueError("必须输入8字节")
            result = unpack_feedback(data)
            position_deg = math.degrees(result['position_rad'])
            velocity_rpm = result['velocity_rad_s'] * (60.0 / (2.0 * math.pi))
            self.decoded.setText(f"ID={result['node_id']} · 位置={position_deg:.2f}° · "
                                 f"速度={velocity_rpm:.2f} rpm · Iq={result['iq_A']:.3f} A · "
                                 f"温度={result['temperature_C']} · 故障={result['error']}")
        except Exception as exc:
            self.decoded.setText("帧无效：" + str(exc))

    def refresh_can_stats(self):
        if not self.can_worker:
            self.decoded.setText("未连接USB设备")
            return
        for pid in range(0x0830, 0x0836):
            self.can_worker.get_param(pid)

    def on_param(self, pid, value):
        if not 0x0830 <= pid <= 0x0835:
            return
        names = {0x0830: "接收帧", 0x0831: "发送帧", 0x0832: "丢弃帧",
                 0x0833: "Bus-off恢复", 0x0834: "控制来源", 0x0835: "CAN请求Iq / A"}
        row = self.diag_table.rowCount()
        for i in range(row):
            if self.diag_table.item(i, 0).text() == names[pid]:
                self.diag_table.item(i, 1).setText(f"{value:g}")
                return
        self.diag_table.insertRow(row)
        self.diag_table.setItem(row, 0, QtWidgets.QTableWidgetItem(names[pid]))
        self.diag_table.setItem(row, 1, QtWidgets.QTableWidgetItem(f"{value:g}"))
