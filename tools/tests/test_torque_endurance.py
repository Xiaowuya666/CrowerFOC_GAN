"""Offline safety regressions only; no device I/O or measured endurance claim."""
import sys
import struct
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from torque_endurance_core import Guard,summarize
from torque_endurance_probe import Capture,MSG_TELEMETRY,encode_frame


def sample(**changes):
    x=dict(t_s=0.,ang=359.,vel=0.,state=3,mode=0,flags=7,fault=0,
           vbus_V=23.,ia_A=.3,ib_A=-.15,ic_A=-.15,id_A=0.,iq_A=.3,
           da=920,db=890,dc=890,stage='hold')
    x.update(changes); return x


class Guards(unittest.TestCase):
    def test_angle_wrap_does_not_look_like_a_full_turn(self):
        g=Guard(1.5,.1,.5)
        self.assertIsNone(g.check(sample(),True))
        self.assertIsNone(g.check(sample(ang=1,t_s=.01),True))
        self.assertAlmostEqual(g.travel,2*3.141592653589793/180)

    def test_fixture_motion_stops_even_if_velocity_is_zero(self):
        g=Guard(1.5,.1,.5)
        g.check(sample(ang=0),True)
        self.assertIn('轴位移',g.check(sample(ang=7,vel=0),True))

    def test_speed_guard_rejects_persistent_excursion_and_big_jump(self):
        g=Guard(1.5,10,.5)
        self.assertIsNone(g.check(sample(vel=2),True))
        self.assertIn('持续',g.check(sample(vel=2,t_s=.06),True))
        g=Guard(1.5,10,.5)
        self.assertIn('硬上限',g.check(sample(vel=4),True))

    def test_raw_outlier_is_not_hidden_by_filtered_current(self):
        g=Guard(1.5,.1,.5)
        self.assertIn('原始',g.check(sample(ia_raw_A=6.6),True))

    def test_unexpected_idle_and_fault_interrupt_drive(self):
        g=Guard(1.5,.1,.5)
        self.assertIn('状态',g.check(sample(state=1),True))
        self.assertIn('故障',g.check(sample(fault=512),True))


class Statistics(unittest.TestCase):
    def test_serial_burst_preserves_every_frame_and_pwm_compares(self):
        rows=[]; capture=Capture('offline',rows.append)
        packets=[]
        for seq in (10,11):
            payload=bytearray(36)
            struct.pack_into('<H',payload,0,seq)
            struct.pack_into('<HHH',payload,24,900,890,910)
            payload[32]=3; payload[33]=0
            packets.append(encode_frame(MSG_TELEMETRY,payload))
        capture._parse(bytearray(b''.join(packets)))
        self.assertEqual([r['seq'] for r in rows],[10,11])
        self.assertEqual([rows[1][k] for k in ('da','db','dc')],[900,890,910])

    def test_idle_historical_dq_cannot_contaminate_hold_result(self):
        rows=[sample(t_s=i*.01) for i in range(200)]
        rows.append(sample(t_s=2,state=1,iq_A=99,stage='stop'))
        result=summarize(rows,.3)
        self.assertTrue(result['valid'])
        self.assertAlmostEqual(result['iq_mean_A'],.3)
        self.assertTrue(result['mean_tracking_passed'])
        self.assertEqual(result['frames'],200)

    def test_incomplete_capture_is_not_a_success(self):
        self.assertFalse(summarize([sample()],.3)['valid'])


if __name__=='__main__': unittest.main()
