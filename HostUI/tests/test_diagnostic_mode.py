import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from diagnostic_mode import inventory, failure_modes, live_checks

class DiagnosticTests(unittest.TestCase):
    def test_every_netlist_reference_has_failure_scenarios(self):
        refs=inventory()
        self.assertGreaterEqual(len(refs),75)
        self.assertIn(('主板','U13'),refs)
        self.assertIn(('第二编码器板','FPC1'),refs)
        for _board,ref in refs:
            self.assertGreaterEqual(len(failure_modes(ref)),2)

    def test_live_checks_reflect_encoder_enable_and_fault(self):
        latest={'fault':0,'vbus':24000,'flags':2,'ia_A':1.0,'ib_A':-0.5,'ic_A':-0.5}
        rows={name:(status,detail) for name,status,detail in live_checks(latest,{0x0840:0},True,True)}
        self.assertEqual(rows['USB遥测'][0],'通过')
        self.assertEqual(rows['母线电压'][0],'通过')
        self.assertEqual(rows['输出轴第二编码器'][0],'通过')
        latest['fault']=4
        rows={name:(status,detail) for name,status,detail in live_checks(latest,{0x0840:1,0x0843:0},True,True)}
        self.assertEqual(rows['驱动故障锁存'][0],'失败/待检查')
        self.assertEqual(rows['输出轴第二编码器'][0],'失败/待检查')

if __name__=='__main__':unittest.main()
