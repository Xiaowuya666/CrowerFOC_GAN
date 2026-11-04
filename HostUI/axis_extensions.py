"""Revision 2 axis parameters, units and offline configuration validation."""
import math
import json

AXIS_FIELDS=('pos_deg','vel_rpm','ia_A','ib_A','ic_A','id_A','iq_A',
             'fet_temp_C','motor_temp_C','bus_current_A','power_W','trajectory_done','command_source','can_armed','measured_gear_ratio')
RPM_PER_RAD_S=60/(2*math.pi)
DEG_PER_RAD=180/math.pi
SPEED_PARAMS={0x0400,0x0402,0x0504,0x0305,0x0310,0x0311,0x0312,0x0214,0x0621,0x0622,0x0846}
ANGLE_PARAMS={0x0200,0x0203,0x0204,0x0205,0x0213,0x0503,0x0410,0x0411,0x0413,0x0415,0x0624,0x0842,0x0845}
BANDWIDTH_PARAMS={0x0107,0x0202}
GAIN_PARAMS={0x0301,0x0302}
RESISTANCE_PARAMS={0x0101}
INDUCTANCE_PARAMS={0x0102}
def parameter_scale(pid):
    if pid in SPEED_PARAMS:return RPM_PER_RAD_S
    if pid in ANGLE_PARAMS:return DEG_PER_RAD
    if pid in BANDWIDTH_PARAMS:return 1/(2*math.pi)
    if pid in GAIN_PARAMS:return 1/RPM_PER_RAD_S
    if pid in RESISTANCE_PARAMS:return 1000.0
    if pid in INDUCTANCE_PARAMS:return 1_000_000.0
    return 1
def from_wire(pid,value):return value*parameter_scale(pid)
def to_wire(pid,value):return value/parameter_scale(pid)
# id, group, label, unit, minimum, maximum; ranges mirror axis_parameters.h.
EXT_PARAMS=[
 (0x0105,'电机','扭矩常数（未知保持0）','Nm/A',0,100),
 (0x0106,'电机','扭矩输入单位（0=A，1=Nm）','',0,1),
 (0x0107,'电机','电流环带宽目标','Hz',50,10000),
 (0x0300,'控制器','位置增益','1/s',0,1000),
 (0x0305,'运动规划','速度斜坡','rad/s²',.01,10000),
 (0x0306,'运动规划','力矩斜坡','A/s 或 Nm/s',.001,10000),
 (0x0307,'运动规划','位置滤波带宽','Hz',.1,100),
 (0x0308,'控制器','电压模型前馈（需Kt）','0/1',0,1),
 (0x0310,'运动规划','轨迹速度限制','rad/s',.01,216.77),
 (0x0311,'运动规划','轨迹加速度','rad/s²',.01,10000),
 (0x0312,'运动规划','轨迹减速度','rad/s²',.01,10000),
 (0x0313,'运动规划','负载惯量前馈','kg·m²',0,1000),
 (0x0410,'轴机械角参考（编码器1/2坐标，依启用配置）','位置下限','rad（机械角）',-1e6,1e6),
 (0x0411,'轴机械角参考（编码器1/2坐标，依启用配置）','位置上限','rad（机械角）',-1e6,1e6),
 (0x0412,'位置参考','启用位置限位','0/1',0,1),
 (0x0413,'轴机械角参考（编码器1/2坐标，依启用配置）','启动相对位置零点','rad（机械角）',-1e6,1e6),
 (0x0414,'位置参考','启用环形位置','0/1',0,1),
 (0x0415,'轴机械角参考（编码器1/2坐标，依启用配置）','环形位置范围','rad（机械角）',.001,1e6),
 (0x0420,'电源保护','欠压跳闸','V',5,30),
 (0x0421,'电源保护','欠压恢复','V',6,32),
 (0x0422,'电源保护','过压跳闸','V',17,34),
 (0x0430,'温度保护','板载温度开始降额','℃',30,100),
 (0x0431,'温度保护','板载过温跳闸','℃',40,120),
 (0x0432,'温度保护','启用电机TEMP2温度保护（PA2；两点PTC拟合）','0/1',0,1),
 (0x0433,'温度保护','绕组开始降额','℃',30,150),
 (0x0434,'温度保护','绕组过温跳闸','℃',40,180),
 (0x0435,'温度保护','TEMP2 PTC线性斜率','Ω/℃',0.1,100),
 (0x0436,'温度保护','TEMP2在25℃时阻值','Ω',100,1e6),
 (0x0437,'温度保护','启用板载NTC温度保护（PA1）','0/1',0,1),
 (0x0440,'电源保护','允许回充电流（估算软限制）','A',0,100),
 (0x0441,'电源保护','输入功率限制（0关闭）','W',0,10000),
 (0x0500,'轴','控制模式（0电流/1速度/2位置）','',0,2),
 (0x0501,'轴','输入模式（1直通/2速度斜坡/3位置滤波/4轨迹/5电流斜坡）','',1,5),
 (0x0503,'轴','目标位置机械角（位置模式需编码器2）','rad（机械角）',-1e6,1e6),
 (0x0800,'CAN','MIT电机节点ID','1~7',1,7),
 (0x0801,'CAN','CAN波特率','kbps',125,1000),
 (0x0840,'编码器2（输出轴）','启用反馈','0/1',0,1),
 (0x0841,'编码器2（输出轴）','方向','±1',-1,1),
 (0x0842,'编码器2（输出轴）','机械角零点偏置','rad（机械角）',-1000000,1000000),
]
EXT_READONLY=[
 (0x0416,'实测映射（编码器1/编码器2）','实测传动比（电机机械角/输出轴机械角；未测为NaN）',''),
 (0x0802,'CAN','CAN自动启动许可（停止后须重新启用）','0/1'),
 (0x0803,'CAN','有效MIT帧自动使能策略','0/1'),
 (0x0804,'CAN','MIT电流字段半量程','A'),
 (0x0805,'CAN','MIT电流量化分辨率','A/count'),
 (0x0810,'运行诊断','板载温度','℃'),(0x0811,'运行诊断','绕组温度（未接=NaN）','℃'),
 (0x0812,'运行诊断','温度降额系数','0~1'),(0x0813,'运行诊断','母线电流估算','A'),
 (0x0814,'运行诊断','电功率估算','W'),(0x0815,'运行诊断','停机原因历史位','bitmask'),
 (0x0816,'运行诊断','轨迹完成','0/1'),
 (0x0820,'配置存储','已从Flash加载','0/1'),(0x0821,'配置存储','保存序号',''),
 (0x0822,'配置存储','保存结果（0无/1成功/2失败）',''),(0x0823,'配置存储','扩展协议版本',''),
 (0x0830,'CAN诊断','收到报文数',''),(0x0831,'CAN诊断','发送报文数',''),
 (0x0832,'CAN诊断','丢弃报文数',''),(0x0833,'CAN诊断','Bus-off重启次数',''),
 (0x0834,'CAN诊断','控制来源（0空闲/1USB/2CAN）',''),(0x0835,'CAN诊断','CAN请求电流','A'),
 (0x0843,'编码器2（输出轴）','状态','0/1'),
 (0x0844,'编码器2（输出轴）','读取失败次数',''),(0x0845,'编码器2（输出轴）','输出轴机械角','rad（机械角）'),
 (0x0846,'编码器2（输出轴）','输出轴机械速度','rad/s'),
 (0x0847,'编码器2（输出轴）','原始角度响应字','uint16'),
 (0x0848,'编码器2（输出轴）','SSC原始安全字','uint16'),
 (0x0849,'编码器2（输出轴）','读取错误位（1发送/2接收/8CRC/16状态）','bitmask'),
 (0x084A,'编码器2（输出轴）','通信及CRC通过次数（独立于角度有效性）',''),
 (0x084B,'编码器2（输出轴）','标定状态（0关闭/1测量/2通信异常/3角度无效/4映射未完成/5完成）',''),
 (0x084C,'编码器2（输出轴）','双编码器运动映射一致性R²','0~1'),
 (0x085F,'电机音乐','播放协议版本','')]
EXT_RANGES={p:(lo,hi) for p,_,_,_,lo,hi in EXT_PARAMS}
EXT_RANGES[0x0400]=(.01,2100/RPM_PER_RAD_S)
EXT_RANGES[0x0402]=(.01,253/RPM_PER_RAD_S)
def display_row(row):
    pid,group,label,unit=row[:4]
    unit=unit.replace('rad/s²','rpm/s').replace('rad/s','rpm').replace('rad','°')
    if pid in BANDWIDTH_PARAMS:unit='Hz'
    if pid in GAIN_PARAMS:unit='A/rpm' if pid==0x0301 else 'A/(rpm·s)'
    return pid,group,label,unit
INTEGER_PARAMS={0x0106,0x0308,0x0412,0x0414,0x0432,0x0437,0x0500,0x0501,0x0800,0x0801,0x0840,0x0841}
def validate_extended(pid,value):
    if not math.isfinite(value): raise ValueError('数值必须有限')
    if pid in EXT_RANGES:
        lo,hi=EXT_RANGES[pid]
        lo,hi=from_wire(pid,lo),from_wire(pid,hi)
        if not lo<=value<=hi: raise ValueError(f'参数范围为 {lo:g}~{hi:g}')
    if pid in INTEGER_PARAMS and value!=int(value): raise ValueError('此参数必须为整数')
    if pid==0x0841 and value not in (-1,1): raise ValueError('输出编码器方向只能为+1或-1')
    if pid==0x0801 and value not in (125,250,500,1000): raise ValueError('波特率必须为125/250/500/1000kbps')
def configuration_document(cache,writable):
    params={f'{p:04X}':float(cache[p]) for p in sorted(writable) if p in cache and p not in (0x0503,0x0504,0x0505,0x0802)}
    if not all(math.isfinite(v) for v in params.values()): raise ValueError('配置包含无效数值')
    return {'format':'CrowerFOC','version':3,'units':'deg,rpm,A,V,degC,mOhm,uH','parameters':params,
            'note':'配置备份不包含有效校准认证，恢复换相参数后必须完整校准；不自动启动'}
def parse_configuration(text,writable):
    doc=json.loads(text)
    if doc.get('format')!='CrowerFOC': raise ValueError('不支持的配置格式/版本/单位')
    version=doc.get('version');units=doc.get('units')
    if not ((version==2 and units=='deg,rpm,A,V,degC') or
            (version==3 and units=='deg,rpm,A,V,degC,mOhm,uH')):
        raise ValueError('不支持的配置格式/版本/单位')
    params=doc.get('parameters')
    if not isinstance(params,dict) or not params: raise ValueError('缺少配置参数')
    result={}
    for key,value in params.items():
        if not isinstance(key,str) or len(key)!=4: raise ValueError('无效参数ID')
        pid=int(key,16)
        if pid not in writable or pid in (0x0503,0x0504,0x0505,0x0802): raise ValueError(f'不允许恢复参数0x{pid:04X}')
        if isinstance(value,bool) or not isinstance(value,(int,float)): raise ValueError('参数必须是数值')
        value=float(value)
        if version==2:
            if pid==0x0101:value*=1000.0
            elif pid==0x0102:value*=1_000_000.0
        validate_extended(pid,value);result[pid]=value
    return result
