"""Execute production C helpers offline in ARM emulation, never through DAP."""
import struct
import subprocess
import tempfile
import unittest
from pathlib import Path
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_R0, UC_ARM_REG_PC

ROOT = Path(__file__).resolve().parents[2]
COMPILER = Path('C:/Program Files/Arm_GNU_Toolchain/bin/arm-none-eabi-gcc.exe')


class ArmControlTests(unittest.TestCase):
    def test_production_current_pi_overspeed_and_calibration_math(self):
        with tempfile.TemporaryDirectory() as folder:
            elf = Path(folder) / 'foc_test.elf'
            result = subprocess.run([str(COMPILER), '-O2', '-mcpu=cortex-m3', '-mthumb',
                                     '-mfloat-abi=soft', '-nostartfiles', '--specs=nosys.specs',
                                     '-Wl,-Ttext=0x10000,-Tdata=0x20000000,-e,test_entry',
                                     str(ROOT / 'tools/tests/cases_foc_limits.c'),
                                     '-lm', '-lc', '-lgcc', '-o', str(elf)], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            data = elf.read_bytes()
        entry, phoff = struct.unpack_from('<II', data, 24)
        phsize, phcount = struct.unpack_from('<HH', data, 42)
        emu = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
        pages = set()
        for i in range(phcount):
            typ, offset, address, _, size, memory, _, _ = struct.unpack_from('<8I', data, phoff+i*phsize)
            if typ != 1 or memory == 0:
                continue
            for page in range(address & ~4095, (address+memory+4095) & ~4095, 4096):
                if page not in pages:
                    emu.mem_map(page, 4096)
                    pages.add(page)
            emu.mem_write(address, data[offset:offset+size])
        emu.mem_map(0x20010000, 0x10000)
        emu.mem_map(0x00FFF000, 4096)
        emu.reg_write(UC_ARM_REG_SP, 0x2001FFF0)
        emu.reg_write(UC_ARM_REG_LR, 0x00FFF001)
        emu.emu_start(entry | 1, 0x00FFF000, timeout=20000000, count=200000000)
        self.assertEqual(emu.reg_read(UC_ARM_REG_PC), 0x00FFF000,
                         f'ARM helper returned early with code {emu.reg_read(UC_ARM_REG_R0)}')
        self.assertEqual(emu.reg_read(UC_ARM_REG_R0), 0, 'C physical/saturation/guard test case failed')


if __name__ == '__main__':
    unittest.main()
