import os
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
import sys
import json
import math
import struct
import time
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from PyQt5 import QtWidgets
from crowerfoc_transport import SerialWorker
from crowerfoc_protocol import *
from crowerfoc_gui import MainWindow
from axis_extensions import configuration_document,parse_configuration,validate_extended
from mit_can import pack_control,unpack_control,special_command,unpack_feedback,CMD_ENABLE,CMD_DISABLE,CMD_ZERO
APP=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
class ExtensionTests(unittest.TestCase):
    def test_mit_can_control_and_special_frame_codec(self):
        command=(1.25,-3.5,80.0,1.2,2.0)
        decoded=unpack_control(pack_control(*command))
        for expected,actual,tol in zip(command,decoded,(.001,.02,.13,.002,.02)):
            self.assertAlmostEqual(actual,expected,delta=tol)
        self.assertEqual(pack_control(-99,99,999,99,99),pack_control(-12.5,30,500,5,40))
        self.assertEqual(pack_control(0,0,0,0,0),bytes((0x80,0,0x80,0,0,0,8,0)))
        self.assertAlmostEqual(unpack_control(pack_control(0,0,0,0,-40))[-1],-40,places=3)
        self.assertAlmostEqual(unpack_control(pack_control(0,0,0,0,40))[-1],40,places=3)
        self.assertEqual(special_command(CMD_ENABLE),bytes((255,))*7+bytes((252,)))
        self.assertEqual(special_command(CMD_DISABLE),bytes((255,))*7+bytes((253,)))
        self.assertEqual(special_command(CMD_ZERO),bytes((255,))*7+bytes((254,)))
        with self.assertRaises(ValueError):pack_control(float('nan'),0,0,0,0)
        with self.assertRaises(ValueError):unpack_control(bytes(7))
        fb=unpack_feedback(bytes((3,0x80,0,0x80,8,0,0xFF,1)))
        self.assertEqual(fb['node_id'],3);self.assertAlmostEqual(fb['position_rad'],0,places=3)
        self.assertAlmostEqual(fb['velocity_rad_s'],0,places=1)
        self.assertAlmostEqual(fb['iq_A'],0,places=1)
        self.assertIsNone(fb['temperature_C']);self.assertEqual(fb['error'],1)

    def test_units_roundtrip_and_speed_has_no_100rad_limit(self):
        for pid,value in ((0x0402,200),(0x0504,200),(0x0503,720),(0x0200,180),(0x0301,.1),(0x0202,50),(0x0101,96.2),(0x0102,39.1)):
            self.assertAlmostEqual(from_wire(pid,to_wire(pid,value)),value)
        self.assertAlmostEqual(to_wire(0x0107,1000),2*math.pi*1000,places=3)
        self.assertAlmostEqual(from_wire(0x0107,6283.185307),1000,places=3)
        self.assertAlmostEqual(to_wire(0x0101,96.2),0.0962,places=6)
        self.assertAlmostEqual(to_wire(0x0102,39.1),0.0000391,places=9)
        self.assertAlmostEqual(from_wire(0x0101,0.0962),96.2,places=4)
        self.assertAlmostEqual(from_wire(0x0102,0.0000391),39.1,places=4)
        validate_extended(0x0107,1000)
        validate_extended(0x0400,2000)
        with self.assertRaises(ValueError):validate_extended(0x0107,1600)
        validate_param(0x0402,235,40);validate_param(0x0103,40,40)
        with self.assertRaises(ValueError):validate_param(0x0103,40.01,40)
        with self.assertRaises(ValueError):validate_param(0x0402,254,40)
    def test_set_and_get_convert_rpm_and_degrees(self):
        worker=SerialWorker('test');values=[];worker.param.connect(lambda p,v:values.append((p,v)))
        worker.set_param(0x0402,200)
        self.assertAlmostEqual(struct.unpack('<f',worker._tx[0][0][6:10])[0],to_wire(0x0402,200),places=3)
        worker._dispatch(MSG_PARAM,struct.pack('<Hf',0x0503,math.pi))
        self.assertAlmostEqual(values[0][1],180,places=4)
        worker.latest={'mode':1};worker.set_setpoint(60)
        self.assertAlmostEqual(struct.unpack('<f',worker._tx[-1][0][4:8])[0],2*math.pi,places=5)
    def test_float_telemetry_preserves_40A_and_continuous_position(self):
        worker=SerialWorker('test')
        values=[2*math.pi,2*math.pi,40,-20,-20,0,40,25,float('nan'),0,0,1,2,1,9.0]
        worker._dispatch(MSG_AXIS_DIAG,struct.pack('<H15f',7,*values))
        p=bytearray(36);struct.pack_into('<H',p,0,7);p[32]=3
        worker._dispatch(MSG_TELEMETRY,p)
        self.assertAlmostEqual(worker.latest['vel'],60,places=4)
        self.assertAlmostEqual(worker.latest['pos'],360,places=2)
        self.assertAlmostEqual(worker.latest['measured_gear_ratio'],9.0)
        self.assertEqual(worker.latest['iq'],40000)
        self.assertTrue(math.isnan(worker.latest['motor_temp_C']))
        p[0]=8;worker._dispatch(MSG_TELEMETRY,p);self.assertNotIn('command_source',worker.latest)
    def test_backup_cannot_restore_targets_or_can_permission(self):
        doc=configuration_document({0x0103:40,0x0402:200,0x0505:30,0x0802:1},WRITABLE_PARAMS)
        self.assertNotIn('0505',doc['parameters']);self.assertNotIn('0802',doc['parameters'])
        self.assertEqual(parse_configuration(json.dumps(doc),WRITABLE_PARAMS)[0x0402],200)
        doc['parameters']['0802']=1
        with self.assertRaises(ValueError):parse_configuration(json.dumps(doc),WRITABLE_PARAMS)
    def test_legacy_config_resistance_units_are_upgraded(self):
        legacy={'format':'CrowerFOC','version':2,'units':'deg,rpm,A,V,degC','parameters':{'0101':0.0962,'0102':0.0000391}}
        restored=parse_configuration(json.dumps(legacy),WRITABLE_PARAMS)
        self.assertAlmostEqual(restored[0x0101],96.2)
        self.assertAlmostEqual(restored[0x0102],39.1)
    def test_new_gui_save_and_can_owner_disable_usb_target(self):
        window=MainWindow();window.timer.stop();worker=SerialWorker('test')
        window.worker=worker;window._connected=True
        p=bytearray(36);struct.pack_into('<H',p,22,24400);p[32]=1;p[35]=6;worker._dispatch(MSG_TELEMETRY,p)
        for pid,val in {0x0100:21,0x0200:20,0x0201:1,0x0103:40,0x0104:5,0x0400:100,0x0500:0,0x0823:2,0x0840:1,0x0843:1,0x0416:9}.items():window.on_param_reply(pid,val)
        window._update_controls();self.assertTrue(window.bt_save.isEnabled());self.assertTrue(window.cb_mode.model().item(2).isEnabled())
        worker.latest.update(state=3,command_source=2);worker.last_telem=time.monotonic();window._update_controls()
        self.assertFalse(window.bt_set.isEnabled());self.assertTrue(window.bt_idle.isEnabled())
        window.worker=None;window.close()
if __name__=='__main__':unittest.main()
