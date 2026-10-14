"""CrowerFOC protocol, firmware capabilities, and parameter validation."""
import math
import struct
from axis_extensions import EXT_PARAMS, EXT_READONLY, AXIS_FIELDS, validate_extended, display_row, RPM_PER_RAD_S, DEG_PER_RAD, from_wire, to_wire

# ----------------------------------------------------------------- 协议
SYNC0, SYNC1 = 0xAA, 0x55
MSG_TELEMETRY, MSG_ACK, MSG_PARAM, MSG_LOG, MSG_INFO = 0x01, 0x02, 0x03, 0x04, 0x05
MSG_CURRENT_RAW = 0x06
MSG_CURRENT_TRACE = 0x07
MSG_CONTROL_DIAG = 0x08
MSG_AXIS_DIAG = 0x0A
MSG_CONFIG_PATCH = 0x89
CONTROL_FIELDS = ('id_target_A', 'iq_target_A', 'vd_V', 'vq_V', 'park_deg',
                  'pwm_deg', 'observer_deg', 'velocity_raw', 'angle_age_us',
                  'voltage_limit_V', 'kcl_raw_A', 'sensor_delay_us')
MSG_HEARTBEAT = 0x88
MSG_SET, MSG_GET, MSG_STATE, MSG_SETPOINT, MSG_SAVE, MSG_CLEAR, MSG_REBOOT = \
    0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87

AXIS_STATES = ["未定义", "空闲", "校准中", "闭环控制", "错误", "参数测量"]
CTRL_MODES = ["力矩", "速度", "位置"]
INPUT_MODES = ["无效", "直通", "速度斜坡", "位置滤波", "梯形轨迹"]

# (id, 分组, 名称, 单位)
PARAMS = [
    (0x0100, "电机", "极对数", ""),
    (0x0101, "电机", "相电阻", "mΩ"),
    (0x0102, "电机", "相电感", "µH"),
    (0x0103, "电机", "电流限制", "A"),
    (0x0104, "电机", "固件标定电流（只读）", "A"),
    (0x0200, "编码器1（电机侧）", "换相电角度零偏", "rad（电角度）"),
    (0x0201, "编码器1（电机侧）", "换相方向", "±1"),
    (0x0202, "编码器1（电机侧）", "机械角观测器带宽", "rad/s"),
    (0x0203, "换相诊断(只读)", "实际Park电角度（编码器1换相）", "rad（电角度）"),
    (0x0204, "换相诊断(只读)", "编码器1电角度", "rad（电角度）"),
    (0x0205, "换相诊断(只读)", "Park电角度误差（编码器1）", "rad（电角度）"),
    (0x0206, "换相诊断(只读)", "编码器1有效机械角数据年龄", "ms；-1=无数据"),
    (0x0207, "换相诊断(只读)", "编码器1机械角/电角度映射一致性", "0~1"),
    (0x0208, "换相诊断(只读)", "标定阶段", "7=完成"),
    (0x0209, "换相诊断(只读)", "SSC安全字", "十进制"),
    (0x020A, "换相诊断(只读)", "编码器1读取失败次数", ""),
    (0x020B, "换相诊断(只读)", "设备标定进度", "%"),
    (0x020C, "换相诊断(只读)", "标定计划时长", "s"),
    (0x020D, "换相诊断(只读)", "R拟合电压截距", "V"),
    (0x020E, "换相诊断(只读)", "R低档实际电流", "A"),
    (0x020F, "换相诊断(只读)", "R低档指令电压", "V"),
    (0x0210, "换相诊断(只读)", "TLE5012B MOD1", "十进制"),
    (0x0211, "换相诊断(只读)", "TLE5012B MOD2", "十进制"),
    (0x0212, "换相诊断(只读)", "芯片角度延迟", "µs"),
    (0x0213, "换相诊断(只读)", "观测器机械角（编码器1）", "rad（机械角）"),
    (0x0214, "换相诊断(只读)", "编码器1原始机械速度", "rad/s"),
    (0x0215, "换相诊断(只读)", "齿槽电流表有效", "0/1"),
    (0x0216, "换相诊断(只读)", "齿槽补偿电流", "A"),
    (0x0220, "电流环测试(只读)", "实测-3dB带宽", "Hz"),
    (0x0221, "电流环测试(只读)", "1kHz电流幅值增益", "倍"),
    (0x0222, "电流环测试(只读)", "1kHz电流相位滞后", "°"),
    (0x0223, "电流环测试(只读)", "1kHz带宽目标通过", "0/1"),
    (0x0300, "控制器", "位置增益（暂未实现）", "1/s"),
    (0x0301, "控制器", "速度增益", "A/(rad/s)"),
    (0x0302, "控制器", "速度积分增益", ""),
    (0x0303, "控制器", "电流环Kp", "V/A"),
    (0x0304, "控制器", "电流环Ki", "V/(A·s)"),
    (0x0400, "限制", "速度指令限制", "rad/s"),
    (0x0401, "限制", "电流限制", "A"),
    (0x0402, "兼容参数", "旧版转速跳闸阈值（当前固件忽略）", "rad/s"),
    (0x0500, "轴", "控制模式", ""),
    (0x0501, "轴", "输入模式", ""),
    (0x0503, "轴", "目标机械角（位置模式需编码器2）", "rad（机械角）"),
    (0x0504, "轴", "目标速度", "rad/s"),
    (0x0505, "轴", "目标Iq", "A"),
    (0x0600, "电流采样(只读)", "原始超过6.45A次数", ""),
    (0x0601, "电流采样(只读)", "原始峰值", "ADC计数"),
    (0x0610, "电流采样(只读)", "ADC异常次数", ""),
    (0x0615, "电流采样(只读)", "A相零点", "ADC计数"),
    (0x0616, "电流采样(只读)", "B相零点", "ADC计数"),
    (0x0617, "电流采样(只读)", "C相零点", "ADC计数"),
    (0x0618, "电流采样(只读)", "中值差超过2A次数", ""),
    (0x0619, "电流采样(只读)", "驱动时ADC最长耗时", "CPU周期/144MHz"),
    (0x061A, "电流采样(只读)", "驱动时ISR最长耗时", "CPU周期/144MHz"),
    (0x0620, "电流采样(只读)", "硬件同步采样", "1=TIM1触发"),
    (0x0621, "控制保护(只读)", "本次运行最高实际转速", "rad/s"),
    (0x0622, "控制保护(只读)", "跳闸时实际转速", "rad/s"),
    (0x0623, "控制保护(只读)", "电流PI电压饱和次数", ""),
    (0x0624, "控制保护(只读)", "PWM作用电角度（依据编码器1）", "rad（电角度）"),
]

WRITABLE_PARAMS = {0x0100, 0x0101, 0x0102, 0x0103, 0x0200, 0x0201,
                   0x0301, 0x0302, 0x0303, 0x0304, 0x0400, 0x0401, 0x0402}
CRITICAL_PARAMS = (0x0100, 0x0200, 0x0201, 0x0103, 0x0104, 0x0400, 0x0500)
SWEEP_FREQS = (100,150,200,300,400,500,650,800,1000,1250,1500,1800,2200,2700,3300,4000)
PARAMS += [(0x0224+i,"电流环测试(只读)",f"{f}Hz幅值增益","倍") for i,f in enumerate(SWEEP_FREQS)]
PARAMS += [(0x0234+i,"电流环测试(只读)",f"{f}Hz相位滞后","°") for i,f in enumerate(SWEEP_FREQS)]
PARAMS += [(0x0244+i,"电流环测试(只读)",f"{f}Hz指令幅度","A") for i,f in enumerate(SWEEP_FREQS)]
PARAMS += [(0x0254+i,"电流环测试(只读)",f"{f}Hz电压饱和占比","%") for i,f in enumerate(SWEEP_FREQS)]
PARAMS += [(0x0264+i,"电流环测试(只读)",f"{f}Hz测量有效","0/1") for i,f in enumerate(SWEEP_FREQS)]
_extended_ids={p for p,*_ in EXT_PARAMS}
PARAMS=[row for row in PARAMS if row[0] not in _extended_ids and row[0] != 0x0402]+[row[:4] for row in EXT_PARAMS]+EXT_READONLY
PARAMS=[display_row(row) for row in PARAMS]
WRITABLE_PARAMS|=_extended_ids
WRITABLE_PARAMS.add(0x0802)
FAULT_NAMES = ("母线过压", "母线欠压", "过流", "功率级过温", "电机过温",
               "编码器异常/数据过期", "换相未配置/标定不一致", "栅极驱动异常", "ADC转换/采样异常", "主机心跳超时", "校准电流跟踪/R或L辨识失败", "旧固件实际转速超限")
FAULT_NAMES+=("位置越界","控制周期超时","急停","配置异常")
CAL_STAGES = {0: "测量电阻", 1: "卸流/电感脉冲", 2: "磁场缓慢启动",
              3: "自动测量极对数与方向", 4: "静态对齐", 5: "正反向偏置扫描",
              6: "独立电流环扫频", 7: "完整标定完成"}
CAL_HINTS = {
    0: "1/6 · 缓升至2.5A和5A，比较两档电压/电流，分离近似固定的压降。轴可能轻微对齐。",
    1: "2/6 · 电流卸载后交替施加16组短脉冲，以斜率回归和去极值平均估算电感；无需操作。",
    2: "3/6 · 5A磁场缓升并启动；电机将缓慢转动，请保持轴自由。",
    3: "4/6 · 扫描16个电周，由编码器1（电机侧）机械角位移计算极对数和方向。",
    4: "5/6 · 暂停旋转并平均编码器1机械角，建立换相电角度偏置。",
    5: "6/6 · 正向、反向各扫描2个电周，检查编码器1与电角度映射一致性并平均偏置。",
    6: "独立扫频：100–4000Hz共16点；转子理想情况下不转，发现微动应立即停止。",
    7: "完整标定结束，设备已停止；扫频需另行启动。齿槽补偿已关闭。",
}
CAL_OPTIONAL_PARAMS = {0x020B, 0x020C, 0x020D, 0x020E, 0x020F,
                       0x0215, 0x0216,
                       0x0402, 0x0621, 0x0622, 0x0623, 0x0624,
                       0x0210, 0x0211, 0x0212, 0x0213, 0x0214,
                       0x0220, 0x0221, 0x0222, 0x0223}
CAL_OPTIONAL_PARAMS.add(0x062A)
CAL_OPTIONAL_PARAMS|=set(_extended_ids)|{p for p,*_ in EXT_READONLY}
WRITABLE_PARAMS.add(0x0840)
CAL_OPTIONAL_PARAMS-=set(CRITICAL_PARAMS)
CAL_HINTS_LEGACY = {
    0: "1/6 · 旧版固件固定0.5A测电阻，可能轻微对齐。",
    1: "2/6 · 卸载电流并测量脉冲响应，无需操作。",
    2: "3/6 · 磁场缓慢启动，请保持轴自由转动。",
    3: "4/6 · 由编码器1（电机侧）机械角位移计算极对数与方向。",
    4: "5/6 · 静态对齐并平均编码器1机械角，建立换相电角度偏置。",
    5: "6/6 · 正反向扫描验证偏置，随后自动停止。",
    6: "换相参数验证完成，电机已停止；正在回读结果。",
}
TELEMETRY_TIMEOUT = 0.7


def encode_frame(typ, payload=b""):
    body = bytes([typ, len(payload)]) + payload
    return b"\xAA\x55" + body + bytes([crc8(body)])


def stop_packet():
    # Cancel every drive path. Send as one ordered transaction, including old targets.
    commands = [(0x01, struct.pack("<H", 0)), (MSG_STATE, b"\x01"),
                (0x09, struct.pack("<H", 0)),
                (MSG_SET, struct.pack("<Hf", 0x0505, 0.0)),
                (MSG_SET, struct.pack("<Hf", 0x0504, 0.0)),
                (MSG_SETPOINT, struct.pack("<f", 0.0))]
    return b"".join(encode_frame(t, p) for t, p in commands)


def validate_param(pid, value, current_limit):
    validate_extended(pid,value)
    if not math.isfinite(value):
        raise ValueError("数值必须有限，不能是NaN或无穷大")
    if pid == 0x0100 and (value != int(value) or not 1 <= value <= 64):
        raise ValueError("极对数必须为1~64的整数，请以测量结果为准")
    if pid == 0x0201 and value not in (-1, 1):
        raise ValueError("编码器方向只能为+1或-1")
    if pid == 0x0202 and not 10/(2*math.pi) <= value <= 800/(2*math.pi):
        raise ValueError("角度观测器带宽必须为1.59~127.32Hz")
    if pid in (0x0103, 0x0401) and not 0 < value <= 40:
        raise ValueError("电流限制必须大于0且不超过40A")
    if pid == 0x0402 and value <= 0:
        raise ValueError("旧版转速跳闸参数必须为正数，单位rpm")
    if pid == 0x0104:
        raise ValueError("标定电流由固件固定配置，无需修改")
    if pid in (0x0101, 0x0102, 0x0400) and value <= 0:
        raise ValueError("此参数必须大于0")
    gain_limits={0x0300:1000,0x0301:10000,0x0302:100000,0x0303:10000,0x0304:1e6}
    if pid in gain_limits and not 0 <= to_wire(pid,value) <= gain_limits[pid]:
        raise ValueError(f"增益超过允许范围")
    if pid == 0x0200 and abs(value) > 360:
        raise ValueError("偏置请使用±360°以内的电角度")

# 遥测载荷偏移（见 proto.h 的 CFF_T_*）
T = dict(seq=0, raw=2, ang=4, ele=6, ia=8, ib=10, ic=12, id=14, iq=16,
         vel=18, pos=20, vbus=22, da=24, db=26, dc=28, fault=30,
         state=32, mode=33, inmode=34, flags=35)
PWM_ARR = 1800.0
MAXW = 4000
WINDOW = 1500


def crc8(d):
    c = 0xFF
    for b in d:
        c ^= b
        for _ in range(8):
            c = ((c << 1) ^ 0x07) & 0xFF if c & 0x80 else (c << 1) & 0xFF
    return c



__all__ = [name for name in globals() if name.isupper()] + ["crc8", "encode_frame", "stop_packet", "validate_param", "from_wire", "to_wire"]
