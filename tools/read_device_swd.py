"""Read application state through SWD without halting, writing, or resuming."""
import json
import struct
import hashlib
import subprocess
import time
import argparse
from datetime import datetime, timezone, timedelta
from pathlib import Path
from pyocd.core.helpers import ConnectHelper
root=Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser()
parser.add_argument('--trace-gate-test',action='store_true')
args=parser.parse_args()
elf=root/'Software/MotorTest_cmake/build/debug_GCC_STM32C562CET6/MotorTest.elf'
nm=subprocess.check_output([r'C:\Program Files\Arm_GNU_Toolchain\bin\arm-none-eabi-nm.exe',str(elf)],text=True)
symbols={x.split()[2]:int(x.split()[0],16) for x in nm.splitlines() if len(x.split())==3}
session=ConnectHelper.session_with_chosen_probe(blocking=False,return_first=True,
 unique_id='846703532218',options={'target_override':'stm32c562cet6','frequency':100000,'connect_mode':'attach'})
if not session:
    raise RuntimeError('Probe missing')
result={'time':datetime.now(timezone(timedelta(hours=8))).isoformat(),'read_only':not args.trace_gate_test,
        'elf_sha256':hashlib.sha256(elf.read_bytes()).hexdigest()}
with session:
    t=session.target
    result['core_state']=str(t.get_state())
    result['tim1_bdtr']=hex(t.read32(0x40012C44))
    result['tim1_trigger_registers']={hex(a):hex(t.read32(a)) for a in
        (0x40012C00,0x40012C04,0x40012C18,0x40012C1C,0x40012C20,
         0x40012C24,0x40012C2C,0x40012C30,0x40012C40,0x40012C44)}
    adc=0x42028000
    result['adc1_trigger_registers']={hex(a):hex(t.read32(a)) for a in
        (adc,adc+8,adc+12,adc+0x4c,adc+0x80,adc+0x84,adc+0x88)}
    result['cfsr']=hex(t.read32(0xE000ED28))
    result['hfsr']=hex(t.read32(0xE000ED2C))
    result['usb_registers']={hex(a):hex(t.read32(a)) for a in range(0x40016040,0x4001605c,4)}
    if '_ux_system_slave' in symbols:
        pointer=t.read32(symbols['_ux_system_slave'])
        result['usbx_pointer']=hex(pointer)
        if 0x20000000<=pointer<0x20020000:
            result['usbx_dcd_words']=t.read_memory_block32(pointer,10)
    for name,width in [('g_ph_state',1),('g_ph_fault',2),('g_ph_enc_ok',1),('g_ph_vbus',2),
                       ('s_enc_config_valid',1),('s_enc_mod1',2),('s_enc_mod2',2),
                       ('s_zero_remaining',4),('g_ph_isr_max',4),('g_ph_adc_to',4),
                       ('g_ph_cnt_upd',2),('g_ph_cnt_rd',2),('g_ph_adc_wait',4),('uwTick',4)]:
        if name in symbols:
            result[name]=t.read_memory(symbols[name],width*8)
    for name in ('s_enc_delay_us','s_vel','s_vel_raw','s_observer_mech'):
        if name in symbols:
            result[name]=struct.unpack('<f',bytes(t.read_memory_block8(symbols[name],4)))[0]
    if args.trace_gate_test:
        if result['g_ph_state']!=1 or int(result['tim1_bdtr'],16)&0x8000:
            raise RuntimeError('Trace gate test requires IDLE and MOE=0')
        original=t.read32(0xE000EDFC)
        def counters():
            return dict(tick=t.read32(symbols['uwTick']),dwt=t.read32(0xE0001004),
                        encoder_count=t.read32(symbols['g_enc_count']),tim2=t.read32(0x40000024))
        try:
            before=counters()
            time.sleep(.05)
            enabled=counters()
            t.write32(0xE000EDFC,original & ~(1<<24))
            off_begin=counters()
            time.sleep(.08)
            off_end=counters()
            t.write32(0xE000EDFC,original | (1<<24))
            on_begin=counters()
            time.sleep(.08)
            on_end=counters()
            result['trace_gate_test']=dict(before=before,enabled=enabled,
                off_begin=off_begin,off_end=off_end,on_begin=on_begin,on_end=on_end)
        finally:
            t.write32(0xE000EDFC,original)
out=root/'tools/fault_results'
out.mkdir(exist_ok=True)
path=out/(datetime.now(timezone(timedelta(hours=8))).strftime('%Y%m%d_%H%M%S')+'_swd.json')
path.write_text(json.dumps(result,indent=2),encoding='utf-8')
print(json.dumps(result))
print('RESULT',path)
