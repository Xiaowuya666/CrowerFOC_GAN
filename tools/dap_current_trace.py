"""Read a frozen 40 kHz capture via DAP without stopping normal firmware."""
import argparse
import json
import struct
import subprocess
import time
from pathlib import Path
from pyocd.core.helpers import ConnectHelper

parser=argparse.ArgumentParser()
parser.add_argument('--diagnose-main',action='store_true')
args=parser.parse_args()
root=Path(__file__).resolve().parents[1]
elf=root/'Software/MotorTest_cmake/build/debug_GCC_STM32C562CET6/MotorTest.elf'
nm=subprocess.check_output([r'C:\Program Files\Arm_GNU_Toolchain\bin\arm-none-eabi-nm.exe',str(elf)],text=True)
symbols={line.split()[2]:int(line.split()[0],16) for line in nm.splitlines() if len(line.split())==3}
session=ConnectHelper.session_with_chosen_probe(blocking=False,return_first=True,
    unique_id='846703532218',options={'target_override':'stm32c562cet6',
    'frequency':250000,'connect_mode':'attach'})
if session is None:
    raise RuntimeError('Probe missing')
with session:
    t=session.target
    if args.diagnose_main:
        if t.read8(symbols['g_ph_state'])!=1 or t.read32(0x40012C44)&0x8000:
            raise RuntimeError('Diagnosis requires IDLE and gates off')
        enabled=t.read32(0x40012C0C)
        try:
            t.write32(0x40012C0C,0)
            t.resume()
            time.sleep(.1)
            t.halt()
            print('MAIN_PC',hex(t.read_core_register('pc')))
            print('MAIN_XPSR',hex(t.read_core_register('xpsr')))
        finally:
            t.write32(0x40012C0C,enabled)
            t.resume()
    else:
        state=t.read16(symbols['s_trace_state'])
        if state!=3:
            raise RuntimeError(f'Trace not frozen: {state}')
        trigger=t.read16(symbols['s_trace_trigger'])
        next_index=t.read16(symbols['s_trace_next'])
        offsets=struct.unpack('<fff',bytes(t.read_memory_block8(symbols['s_i_off'],12)))
        payload=bytes(t.read_memory_block8(symbols['s_current_trace'],64*28))
        samples=[]
        for k in range(64):
            index=(next_index+k)&63
            adc_a,adc_b,adc_c,flags,start,end,min_ccr,direction,ia,ib,ic=struct.unpack_from('<8H3f',payload,index*28)
            delta=(index-trigger)&63
            if delta>16: delta-=64
            samples.append({'t_us':delta*25,'raw_adc':[adc_a,adc_b,adc_c],
                'raw_A':[(v-o)*.03223 for v,o in zip((adc_a,adc_b,adc_c),offsets)],
                'conditioned_A':[ia,ib,ic],'flags':flags,
                'tim_start':start,'tim_end':end,'min_ccr':min_ccr,'down':direction})
        report={'trigger':trigger,'next':next_index,'offsets':offsets,'samples':samples}
        destination=root/'tools/current_noise_results/frozen_current_trace.json'
        destination.write_text(json.dumps(report,indent=2),encoding='utf-8')
        print('TRACE',destination)
        for sample in samples:
            if -75<=sample['t_us']<=75:
                print(json.dumps(sample))
