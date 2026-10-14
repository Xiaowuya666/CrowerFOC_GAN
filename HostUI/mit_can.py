"""MIT-compatible classic CAN bit packing with current-only CrowerFOC units."""
import math

P_MIN, P_MAX = -12.5, 12.5
V_MIN, V_MAX = -30.0, 30.0
KP_MIN, KP_MAX = 0.0, 500.0
KD_MIN, KD_MAX = 0.0, 5.0
I_MIN, I_MAX = -40.0, 40.0
CURRENT_FULL_SCALE_A = 40.0
CMD_ENABLE, CMD_DISABLE, CMD_ZERO = 0xFC, 0xFD, 0xFE


def _uint(value, lo, hi, bits):
    value = float(value)
    if not math.isfinite(value):
        raise ValueError("MIT命令数值必须有限")
    value = min(hi, max(lo, value))
    return round((value - lo) * ((1 << bits) - 1) / (hi - lo))


def _float(value, lo, hi, bits):
    return value * (hi - lo) / ((1 << bits) - 1) + lo


def pack_control(position_rad, velocity_rad_s, kp_a_per_rad, kd_a_per_rad_s, iq_ff_A):
    p = _uint(position_rad, P_MIN, P_MAX, 16)
    v = _uint(velocity_rad_s, V_MIN, V_MAX, 12)
    kp = _uint(kp_a_per_rad, KP_MIN, KP_MAX, 12)
    kd = _uint(kd_a_per_rad_s, KD_MIN, KD_MAX, 12)
    current = _uint(iq_ff_A, I_MIN, I_MAX, 12)
    return bytes((p >> 8, p & 0xFF, v >> 4,
                  ((v & 0xF) << 4) | (kp >> 8), kp & 0xFF,
                  kd >> 4, ((kd & 0xF) << 4) | (current >> 8), current & 0xFF))


def unpack_control(data):
    if len(data) != 8:
        raise ValueError("MIT控制帧必须为8字节")
    p, v = (data[0] << 8) | data[1], (data[2] << 4) | (data[3] >> 4)
    kp = ((data[3] & 0xF) << 8) | data[4]
    kd = (data[5] << 4) | (data[6] >> 4)
    current = ((data[6] & 0xF) << 8) | data[7]
    return (_float(p, P_MIN, P_MAX, 16), _float(v, V_MIN, V_MAX, 12),
            _float(kp, KP_MIN, KP_MAX, 12), _float(kd, KD_MIN, KD_MAX, 12),
            _float(current, I_MIN, I_MAX, 12))


def special_command(opcode):
    if opcode not in (CMD_ENABLE, CMD_DISABLE, CMD_ZERO):
        raise ValueError("不支持的MIT特殊命令")
    return b"\xff" * 7 + bytes((opcode,))


def unpack_feedback(data):
    """Decode reply: id,p16,v12,Iq12,temp+40,error."""
    if len(data) != 8:
        raise ValueError("MIT反馈帧必须为8字节")
    p = (data[1] << 8) | data[2]
    v = (data[3] << 4) | (data[4] >> 4)
    iq = ((data[4] & 0xF) << 8) | data[5]
    temp = None if data[6] == 0xFF else data[6] - 40
    return {"node_id": data[0], "position_rad": _float(p, P_MIN, P_MAX, 16),
            "velocity_rad_s": _float(v, V_MIN, V_MAX, 12),
            "iq_A": _float(iq, I_MIN, I_MAX, 12),
            "temperature_C": temp, "error": data[7]}
