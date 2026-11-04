import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
import sys
import struct
import time
import unittest
import numpy as np
import pyqtgraph as pg
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from PyQt5 import QtCore, QtWidgets
from crowerfoc_gui import MainWindow
from crowerfoc_transport import SerialWorker, HEARTBEAT_INTERVAL
from crowerfoc_protocol import *

APP = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


def telemetry(state=1, flags=6, angle=35999, seq=0, fault=0):
    p = bytearray(36)
    struct.pack_into("<HH", p, 0, seq, 32767)
    struct.pack_into("<HH", p, 4, angle, angle)
    struct.pack_into("<H", p, 22, 23600)
    struct.pack_into("<H", p, 30, fault)
    p[32], p[33], p[34], p[35] = state, 0, 1, flags
    return p


class TransportTests(unittest.TestCase):
    def test_heartbeat_continues_when_telemetry_is_stale(self):
        class FakeSerial:
            def __init__(self):
                self.writes = []
            def write(self, packet):
                self.writes.append(bytes(packet))
                return len(packet)

        r = SerialWorker("test")
        r.sp = FakeSerial()
        r.last_telem = 1.0
        next_heartbeat = r._heartbeat_tick(10.0, 0.0)
        self.assertEqual(r.sp.writes, [encode_frame(MSG_HEARTBEAT)])
        self.assertEqual(next_heartbeat, 10.0 + HEARTBEAT_INTERVAL)
        r.last_telem = 0.0
        next_heartbeat = r._heartbeat_tick(next_heartbeat, next_heartbeat)
        self.assertEqual(len(r.sp.writes), 2)
        self.assertEqual(r.sp.writes[-1], encode_frame(MSG_HEARTBEAT))

    def test_encoder_pair_packet_is_sequence_matched_and_unit_converted(self):
        r=SerialWorker('test')
        values=(.1,.2,.3,.4,.5,.6,.7,9.5)
        r._dispatch(MSG_ENCODERS,struct.pack('<H8fBB',7,*values,15,5))
        r._dispatch(MSG_TELEMETRY,telemetry(seq=7))
        self.assertAlmostEqual(r.latest['enc1_ang'],.1*180/3.141592653589793,places=6)
        self.assertAlmostEqual(r.latest['enc2_ang'],.5*180/3.141592653589793,places=6)
        self.assertAlmostEqual(r.latest['enc2_pos'],.6*180/3.141592653589793,places=5)
        self.assertAlmostEqual(r.latest['enc2_vel'],.7*60/6.283185307179586,places=6)
        self.assertAlmostEqual(r.latest['encoder_ratio'],9.5)
        self.assertEqual(r.latest['encoder2_calibration'],5)
        r._dispatch(MSG_ENCODERS,struct.pack('<H8fBB',8,*values,15,5))
        r._dispatch(MSG_TELEMETRY,telemetry(seq=9))
        self.assertTrue(np.isnan(r.latest['enc2_ang']))

    def test_control_diagnostics_require_matching_sequence_and_finite_values(self):
        r = SerialWorker('test')
        values = [float(i) for i in range(12)]
        r._dispatch(MSG_CONTROL_DIAG, struct.pack('<H12f',7,*values))
        r._dispatch(MSG_TELEMETRY, telemetry(seq=7))
        self.assertEqual(r.latest['vd_V'],2)
        self.assertEqual(r.latest['sensor_delay_us'],11)
        r._dispatch(MSG_CONTROL_DIAG, struct.pack('<H12f',8,*values))
        r._dispatch(MSG_TELEMETRY, telemetry(seq=9))
        self.assertNotIn('vd_V',r.latest)
        values[2]=float('nan')
        r._dispatch(MSG_CONTROL_DIAG, struct.pack('<H12f',10,*values))
        r._dispatch(MSG_TELEMETRY, telemetry(seq=10))
        self.assertNotIn('vd_V',r.latest)
    def test_write_removes_old_queued_read_so_readback_follows_write(self):
        r = SerialWorker("test")
        r.get_param(0x0301)
        r.set_param(0x0301, .15)
        r.get_param(0x0301)
        self.assertEqual([p[0][2] for p in r._tx], [MSG_SET, MSG_GET])
    def test_raw_current_is_aligned_and_does_not_wrap_above_32A(self):
        r = SerialWorker("test")
        r._dispatch(MSG_CURRENT_RAW, struct.pack("<HHHHfffH", 7, 3600, 1200, 1200, 50, -25, -25, 1))
        r._dispatch(MSG_TELEMETRY, telemetry(seq=7))
        self.assertEqual(r.latest["ia_raw"], 50000)
        self.assertEqual(r.latest["adc_raw"], [3600, 1200, 1200])
        r._dispatch(MSG_CURRENT_RAW, struct.pack("<HHHHfffH", 8, 2000, 2000, 2000, 1, 1, 1, 0))
        r._dispatch(MSG_TELEMETRY, telemetry(seq=9))
        self.assertNotIn("ia_raw", r.latest)
        self.assertTrue(__import__('numpy').isnan(r.snapshot(1)["ia_raw"][0]))
    def test_unsigned_angles_and_sequence_reset(self):
        r = SerialWorker("test")
        r._dispatch(MSG_TELEMETRY, telemetry(seq=200))
        self.assertAlmostEqual(r.latest["ang"], 359.99, places=3)
        r._dispatch(MSG_TELEMETRY, telemetry(seq=0))
        self.assertEqual(r.missed, 0)

    def test_corrupt_length_and_crc_recover_next_frame(self):
        r = SerialWorker("test")
        valid = encode_frame(MSG_TELEMETRY, telemetry())
        corrupt = bytearray(valid)
        corrupt[-1] ^= 1
        b = bytearray(b"noise\xaa\x55\x01\xff" + corrupt + valid[:9])
        r._parse(b)
        self.assertEqual(r.count, 0)
        b.extend(valid[9:])
        r._parse(b)
        self.assertEqual(r.count, 1)

    def test_stop_supersedes_queued_drive_and_reads(self):
        r = SerialWorker("test")
        r.set_setpoint(.6)
        r.get_param(0x0100)
        r.safe_idle()
        self.assertEqual(len(r._tx), 1)
        self.assertEqual(r._tx[0][0], stop_packet())
        r.get_param(0x0100)
        r.get_param(0x0100)
        self.assertEqual(len(r._tx), 2)

    def test_snapshot_is_independent_of_writer(self):
        r = SerialWorker("test")
        r._dispatch(MSG_TELEMETRY, telemetry())
        sample = r.snapshot(1)
        r.data["ang"][0] = 1
        self.assertAlmostEqual(sample["ang"][0], 359.99, places=3)


class WorkflowTests(unittest.TestCase):
    def setUp(self):
        self.w = MainWindow()
        self.w.timer.stop()
        self.r = SerialWorker("test")
        self.w.worker = self.r
        self.w._connected = True
        self.r._dispatch(MSG_TELEMETRY, telemetry())
        for p, v in {0x0100: 21, 0x0200: .28, 0x0201: 1,
                     0x0103: 1, 0x0104: 1, 0x0400: 10, 0x0500: 0,0x062A:2}.items():
            self.w.on_param_reply(p, v)
        self.w._update_device()

    def tearDown(self):
        for dialog in list(self.w._dialogs):
            dialog.close()
        self.w.worker = None
        self.w.close()
        APP.processEvents()

    def test_device_sync_never_writes_and_idle_disables_target(self):
        self.assertEqual(len(self.r._tx), 0)
        self.assertFalse(self.w.bt_set.isEnabled())
        self.assertFalse(self.w.target_group.isEnabled())
        self.assertTrue(self.w.bt_cal.isEnabled())
        self.assertTrue(self.w.bt_closed.isEnabled())
        self.assertFalse(self.w.bt_save.isEnabled())
        self.assertEqual(self.w.cb_pid.findData(0x0207), -1)

    def test_temperature_protection_switches_are_explicit_and_writeable_when_idle(self):
        self.w.on_param_reply(0x0823, 2)
        self.w.on_param_reply(0x0432, 1)
        self.w.on_param_reply(0x0437, 1)
        self.w._update_controls()
        self.assertTrue(self.w.ck_motor_temp.isChecked())
        self.assertTrue(self.w.ck_fet_temp.isChecked())
        self.assertTrue(self.w.ck_motor_temp.isEnabled())
        self.w.ck_motor_temp.setChecked(False)
        set_frames = [f for f, _ in self.r._tx if f[2] == MSG_SET]
        self.assertEqual(len(set_frames), 1)
        self.assertEqual(struct.unpack_from('<H', set_frames[0], 4)[0], 0x0432)
        self.assertEqual(struct.unpack_from('<f', set_frames[0], 6)[0], 0.0)

    def test_unknown_mapping_blocks_closed_but_allows_combined_calibration(self):
        self.w.on_param_reply(0x0100, 0)
        self.w._update_controls()
        self.assertFalse(self.w.bt_closed.isEnabled())
        self.assertTrue(self.w.bt_cal.isEnabled())
        self.w.cmd_state(3)
        self.assertEqual(len(self.r._tx), 0)
        self.w.cmd_state(2)
        self.assertEqual(self.r._tx[-1][0], encode_frame(MSG_STATE, b"\x02"))

    def test_calibration_current_above_limit_is_explained_and_blocked(self):
        self.w.on_param_reply(0x0104, 1.5)
        self.w._update_controls()
        self.assertFalse(self.w.bt_cal.isEnabled())
        self.assertIn("标定电流超过", self.w.state_banner.text())

    def test_fixed_profile_calibration_does_not_raise_running_current_limit(self):
        self.w.on_param_reply(0x0104, 5)
        self.w.on_param_reply(0x020C, 14.5)
        self.w._update_controls()
        self.assertTrue(self.w.bt_cal.isEnabled())
        self.assertFalse(self.w.sp_cal_cur.isEnabled())
        self.assertEqual(self.w.cb_pid.findData(0x0104), -1)
        self.w.cmd_state(2)
        self.assertEqual(self.w.param_cache[0x0103], 1)
        self.assertEqual(self.r._tx[-1][0], encode_frame(MSG_STATE, b'\x02'))

    def test_applying_running_limits_never_writes_calibration_current(self):
        self.w.on_param_reply(0x0104, 5)
        self.w.on_param_reply(0x020C, 14.5)
        self.w.sp_limit_cur.setValue(.5)
        self.w.sp_limit_vel.setValue(4)
        self.w.apply_limits()
        ids = [struct.unpack_from('<H', frame, 4)[0] for frame, _ in self.r._tx if frame[2] == MSG_SET]
        self.assertEqual(ids, [0x0103, 0x0400])

    def test_device_progress_and_stage_hint_override_host_elapsed_time(self):
        self.w.on_param_reply(0x0104, 5)
        self.w.on_param_reply(0x020C, 14.5)
        self.w.on_param_reply(0x0220, 0)
        self.w.cmd_state(2)
        self.r._dispatch(MSG_TELEMETRY, telemetry(state=2, flags=2))
        self.w._cal_started = time.monotonic() - 100
        self.w.on_param_reply(0x0208, 3)
        self.w.on_param_reply(0x020B, 42)
        self.w._update_device()
        self.assertEqual(self.w.progress.value(), 42)
        self.assertIn('4/6', self.w.lb_cal_hint.text())
        self.assertIn('16个电周', self.w.lb_cal_hint.text())
        self.assertFalse(self.w.bt_closed.isEnabled())
        self.assertIn('极对数', self.w.log.toPlainText())

    def test_optional_new_firmware_parameters_do_not_block_old_firmware_readiness(self):
        self.w.read_all_params()
        self.assertFalse(self.w._reads & CAL_OPTIONAL_PARAMS)
        for pid in list(self.w._reads):
            self.w.on_param_reply(pid, self.w.param_cache.get(pid, 1))
        self.assertTrue(self.w._ready())

    def test_new_profile_waits_for_device_progress_instead_of_predicting_completion(self):
        self.w.on_param_reply(0x0104, 5)
        self.w.on_param_reply(0x020C, 14.5)
        self.w.cmd_state(2)
        self.r._dispatch(MSG_TELEMETRY, telemetry(state=2, flags=2))
        self.w._cal_started = time.monotonic() - 100
        self.w._update_device()
        self.assertEqual(self.w.progress.maximum(), 0)
        self.assertFalse(self.w._dialogs)

    def test_fault_disables_start_and_allows_clear_without_rearming(self):
        self.r._dispatch(MSG_TELEMETRY, telemetry(fault=256))
        self.w._update_device()
        self.assertFalse(self.w.bt_cal.isEnabled())
        self.assertFalse(self.w.bt_closed.isEnabled())
        self.assertFalse(self.w.target_group.isEnabled())
        self.assertTrue(self.w.bt_clear.isEnabled())
        self.w.bt_clear.click()
        self.assertNotIn(encode_frame(MSG_STATE, b'\x03'), [p for p,_ in self.r._tx])

    def test_actual_speed_trip_is_removed_but_velocity_reference_limit_remains(self):
        self.w.on_param_reply(0x0400, 230)
        self.w.on_param_reply(0x0402, 235)
        self.w._update_controls()
        self.assertFalse(hasattr(self.w, 'sp_actual_speed'))
        self.assertEqual(self.w.sp_limit_vel.value(), 230)
        self.w.apply_limits()
        ids = [struct.unpack_from('<H', p, 4)[0] for p, _ in self.r._tx if p[2] == MSG_SET]
        self.assertEqual(ids, [0x0103, 0x0400])

    def test_velocity_mode_uses_motor_encoder_without_output_encoder(self):
        self.w._update_controls()
        self.assertTrue(self.w.cb_mode.model().item(1).isEnabled())
        self.assertFalse(self.w.cb_mode.model().item(2).isEnabled())
        self.assertEqual(self.w.sp_limit_vel.maximum(), 2100.0)

    def test_current_loop_sweep_is_a_separate_calibrated_operation(self):
        self.w._update_controls()
        self.assertTrue(self.w.bt_bw.isEnabled())
        self.w.cmd_bw_sweep()
        set_ids=[struct.unpack_from('<H',p,4)[0] for p,_ in self.r._tx if p[2]==MSG_SET]
        self.assertEqual(set_ids,[0x061E])
        self.assertEqual(self.w._active_cal_kind,"sweep")

    def test_old_firmware_disables_new_response_tests_without_parameter_spam(self):
        self.w.on_param_reply(0x062A,float("nan"))
        self.w._update_controls()
        self.assertFalse(self.w.bt_bw.isEnabled())
        self.w.read_all_params()
        requested=[struct.unpack('<H',p[4:6])[0] for p,pid in self.r._tx if p[2]==MSG_GET]
        self.assertFalse(any(0x0220<=pid<0x0274 for pid in requested))

    def test_bode_result_builds_amplitude_and_phase_plots(self):
        self.w._result_data={0x0220:1000,0x0223:1}
        self.w._result_data.update({pid:1.0 for pid in range(0x0224,0x0234)})
        self.w._result_data.update({pid:20.0 for pid in range(0x0234,0x0244)})
        self.w._result_data[0x0234+14]=210.0
        self.w._result_data.update({pid:2.0 for pid in range(0x0244,0x0254)})
        self.w._result_data.update({pid:0.0 for pid in range(0x0254,0x0264)})
        self.w._result_data.update({pid:1.0 for pid in range(0x0264,0x0274)})
        self.w.show_bode_result()
        dialog=self.w._dialogs[-1]
        self.assertEqual(dialog.windowTitle(),"电流环扫频 · 波特图")
        self.assertEqual(len(dialog.findChildren(pg.PlotWidget)),2)
        phase_line=dialog.findChildren(pg.PlotWidget)[1].listDataItems()[0]
        _,phase=phase_line.getData()
        self.assertAlmostEqual(phase[14],-210.0)

    def test_bode_keeps_invalid_finite_points_visible_and_table_is_bounded(self):
        self.w._result_data={0x0220:900,0x0223:0}
        self.w._result_data.update({pid:1.0 for pid in range(0x0224,0x0234)})
        self.w._result_data.update({pid:30.0 for pid in range(0x0234,0x0244)})
        self.w._result_data.update({pid:1.0 for pid in range(0x0244,0x0254)})
        self.w._result_data.update({pid:0.0 for pid in range(0x0254,0x0264)})
        self.w._result_data.update({pid:1.0 for pid in range(0x0264,0x0274)})
        self.w._result_data[0x0267]=0.0
        self.w.show_bode_result()
        dialog=self.w._dialogs[-1]
        self.assertEqual(len(dialog.findChildren(QtWidgets.QTableWidget)),1)
        plot=dialog.findChildren(pg.PlotWidget)[0]
        invalid=[item for item in plot.listDataItems() if item.opts.get("symbol")=="x"]
        self.assertEqual(len(invalid),1)
        x, y=invalid[0].getData()
        self.assertEqual(len(y),1)
        self.assertTrue(np.isfinite(y[0]))
        self.assertAlmostEqual(x[0],np.log10(300),places=6)

    def test_torque_slider_streams_current_and_zeroes_on_release(self):
        self.r._dispatch(MSG_TELEMETRY,telemetry(state=3,flags=7))
        self.w._update_device();self.w._update_controls()
        self.w.sl_torque.setValue(1000)
        self.w._torque_slider_pressed()
        self.assertTrue(self.w._torque_drag)
        self.assertAlmostEqual(self.w._torque_slider_current(),.8)
        self.w._torque_slider_released()
        self.assertFalse(self.w._torque_drag)
        self.assertEqual(self.w.sl_torque.value(),0)
        self.assertEqual(self.r._tx[-1][0],encode_frame(MSG_SETPOINT,struct.pack('<f',0)))

    def test_second_encoder_enable_is_a_verified_parameter_write(self):
        self.w.on_param_reply(0x0840,0)
        self.w.on_param_reply(0x0843,1)
        self.w._update_controls()
        self.w.ck_encoder2.setChecked(True)
        sent=[struct.unpack_from('<Hf',p,4) for p,_ in self.r._tx if p[2]==MSG_SET]
        self.assertTrue(sent)
        self.assertEqual(sent[-1][0],0x0840)

    def test_overspeed_fault_is_identified_without_automatic_rearming(self):
        self.r._dispatch(MSG_TELEMETRY, telemetry(fault=2048))
        self.w._update_device()
        self.assertIn('实际转速超限', self.w.lb_fault.text())
        self.assertFalse(self.w.bt_closed.isEnabled())
        self.assertFalse(self.w.target_group.isEnabled())
        self.assertTrue(self.w.bt_clear.isEnabled())

    def test_closed_loop_starts_with_zero_and_waits_for_telemetry(self):
        self.w.sp_set.setValue(.8)
        self.w.cmd_state(3)
        self.assertEqual(self.w.sp_set.value(), 0)
        self.assertEqual(self.r._tx[0][0], stop_packet())
        self.assertFalse(self.w.bt_set.isEnabled())
        self.r._dispatch(MSG_TELEMETRY, telemetry(state=3, flags=7))
        self.w._update_device()
        self.assertTrue(self.w.bt_set.isEnabled())
        self.assertTrue(self.w.target_group.isEnabled())
        self.assertFalse(self.w.cb_mode.isEnabled())
        self.assertFalse(self.w.bt_pset.isEnabled())

    def test_write_readback_and_rejected_value(self):
        self.w.do_set(0x0103, .5)
        self.assertFalse(self.w.bt_closed.isEnabled())
        self.w.on_param_reply(0x0103, 1)
        self.assertIn("未生效", self.w.lb_param_status.text())
        self.assertEqual(self.w.param_cache[0x0103], 1)
        self.w.do_set(0x0201, 0)
        self.assertNotIn(0x0201, self.w._writes)

    def test_initial_sync_blocks_writes_until_all_replies_arrive(self):
        self.w._reads = {0x0301}
        self.w._update_controls()
        self.assertFalse(self.w.bt_pset.isEnabled())
        self.w.do_set(0x0301, .15)
        self.assertFalse(self.w._writes)
        self.w.on_param_reply(0x0301, .05)
        self.assertTrue(self.w._ready())

    def test_stale_telemetry_stops_and_disables_actions(self):
        self.r.last_telem = time.monotonic() - 2
        self.w._update_device()
        self.assertFalse(self.w.bt_closed.isEnabled())
        self.assertFalse(self.w.bt_cal.isEnabled())
        self.assertEqual(self.r._tx[0][0], stop_packet())
        self.assertIn("失联", self.w.state_banner.text())

    def test_idle_dq_history_is_hidden_and_vector_rejects_common_mode(self):
        for seq, state in ((10, 3), (11, 1)):
            p = telemetry(state=state, seq=seq)
            struct.pack_into('<5h', p, 8, 1000, 1000, 1000, 900, 800)
            self.r._dispatch(MSG_TELEMETRY, p)
        self.w.tick()
        _, current = self.w.c_dq['id'].getData()
        self.assertTrue(np.isnan(current[-1]))
        self.assertAlmostEqual(current[-2], .9)
        _, live_iq = self.w.c_dq_live['iq'].getData()
        self.assertAlmostEqual(live_iq[-2], .8)
        alpha, beta = self.w.v_traj.getData()
        self.assertEqual(alpha[-1], 0)
        self.assertEqual(beta[-1], 0)

    def test_live_monitor_keeps_key_plots_outside_workspace_tabs_and_uses_amperes(self):
        p=telemetry(state=3,seq=11)
        struct.pack_into('<5h',p,8,1250,-500,-750,250,1000)
        self.r._dispatch(MSG_TELEMETRY,p)
        self.w.tick()
        self.assertIs(self.w.main_splitter.widget(0),self.w.live_monitor)
        self.assertIs(self.w.main_splitter.widget(1),self.w.workspaces)
        self.assertEqual(self.w.workspaces.count(),6)
        self.assertEqual(len(self.w.pw_mot2.listDataItems()),1)
        self.assertEqual(len(self.w.pw_vel2.listDataItems()),1)
        self.assertEqual(len(self.w.pw_pos2.listDataItems()),1)
        self.assertEqual(self.w.lb_rate.text(), "速率 --")
        _, phase_a=self.w.c_cur_live['ia'].getData()
        _, current_d=self.w.c_dq_live['id'].getData()
        _, encoder_angle=self.w.c_enc_live['ang'].getData()
        self.assertAlmostEqual(phase_a[-1],1.25)
        self.assertAlmostEqual(current_d[-1],.25)
        self.assertAlmostEqual(encoder_angle[-1],359.99, places=4)
        self.assertIs(self.w.control_splitter.widget(0), self.w.control_panel)
        self.assertFalse(self.w.control_panel.findChildren(QtWidgets.QScrollArea))
        self.assertEqual(self.w.sp_limit_cur.value(), 1.0)  # device readback overrides reference default
        self.assertEqual(self.w.sp_limit_vel.value(), 10.0)  # device readback overrides reference default
        self.assertIn("已连接", self.w.readout_labels["device"].text())
        for item in (self.w.c_cur_live["ia"], self.w.c_dq_live["iq"], self.w.c_enc_live["ang"]):
            self.assertEqual(item.__class__.__name__, "PlotDataItem")
            self.assertIsNone(item.opts["symbol"])
            self.assertEqual(item.opts["pen"].style(), QtCore.Qt.SolidLine)

    def test_disconnected_view_has_explicit_placeholder_and_no_minimum_limit_values(self):
        self.w.tick()
        self.assertGreater(len(self.w.c_cur_live["ia"].getData()[0]), 0)
        self.w._connected = False
        self.w._update_control_readouts()
        self.w._set_live_plot_state(False)
        self.assertEqual(self.w.readout_labels["device"].text(), "未连接")
        self.assertIn("等待连接", self.w.pw_cur_live.plotItem.titleLabel.text)
        self.assertIsNone(self.w.c_cur_live["ia"].getData()[0])
        self.assertGreater(self.w.sp_limit_cur.value(), 0.05)

    def test_phase_display_rejects_common_mode_only_in_conditioned_view(self):
        s={name:np.array([value],dtype=np.float32) for name,value in
           {'ia':1200,'ib':300,'ic':300,'ia_raw':1200,'ib_raw':300,'ic_raw':300}.items()}
        conditioned=MainWindow._phase_plot_values(s)
        raw=MainWindow._phase_plot_values(s,raw=True)
        self.assertAlmostEqual(float(conditioned.sum()),0,places=6)
        self.assertAlmostEqual(float(conditioned[0,0]),.6,places=5)
        self.assertAlmostEqual(float(raw[0,0]),1.2,places=5)
        self.assertAlmostEqual(float(raw.sum()),1.8,places=5)

    def test_progress_and_popup_only_after_results_read_back(self):
        self.w.cmd_state(2)
        self.r._dispatch(MSG_TELEMETRY, telemetry(state=2, flags=2))
        self.w._cal_started = time.monotonic() - 12
        self.w._update_device()
        self.assertGreater(self.w.progress.value(), 30)
        self.assertFalse(self.w.bt_closed.isEnabled())
        self.r._dispatch(MSG_TELEMETRY, telemetry(state=1, flags=6))
        self.w._update_device()
        self.assertEqual(self.w.progress.value(), 100)
        self.assertFalse(self.w._dialogs)
        self.assertTrue({0x0303, 0x0304}.issubset(self.w._result_wait))
        for p in list(self.w._result_wait - {0x0303, 0x0304}):
            self.w.on_param_reply(p, .95 if p == 0x0207 else 1)
        self.assertFalse(self.w._dialogs)
        self.w.on_param_reply(0x0303, .179)
        self.assertFalse(self.w._dialogs)
        self.w.on_param_reply(0x0304, 145.4)
        self.assertEqual(len(self.w._dialogs), 1)
        self.assertEqual(self.w._dialogs[0].windowTitle(), "完整标定结果")

    def test_cancelled_calibration_reports_cancellation(self):
        self.w.cmd_state(2)
        self.r._dispatch(MSG_TELEMETRY, telemetry(state=2, flags=2))
        self.w._update_device()
        self.w.cmd_state(1)
        self.r._dispatch(MSG_TELEMETRY, telemetry(state=1, flags=2))
        self.w._update_device()
        self.assertEqual(self.w._dialogs[0].windowTitle(), "完整标定未完成")


if __name__ == "__main__":
    unittest.main()
