#!/usr/bin/env python3
import unittest
import os
from cffi import FFI

from selfdrive.controls.controlsd_cc import libcontrolsd_cc
from selfdrive.car.toyota.values import CAR
from selfdrive.car.toyota.interface import CarInterface

from test_process_replay import get_segment
from tools.lib.logreader import LogReader
from selfdrive.car.toyota.carstate import get_can_parser


class TestCanParser(unittest.TestCase):
  def test_cc_equal_python(self):
    CP = CarInterface.get_params(CAR.COROLLAH_TSS2)
    ffi = FFI()
    car_params_data = ffi.new('unsigned char[]', CP.to_bytes())
    controls_dir = os.path.dirname((os.path.dirname(os.path.abspath(__file__))))
    libdbc_fn = ffi.new('char[]', os.path.join(os.path.dirname(controls_dir), 'can', "libdbc.so").encode('ascii'))

    segment = "cce908f7eb8db67d|2019-08-02--15-09-51--3"
    tmp_rlog_fn = get_segment(segment)
    lr = LogReader(tmp_rlog_fn)
    os.remove(tmp_rlog_fn)
    all_msgs = sorted(lr, key=lambda msg: msg.logMonoTime)
    can_msgs = [msg for msg in all_msgs if msg.which() == 'can']
    pcp = get_can_parser(CP)
    cp = libcontrolsd_cc.test_canparser_init(libdbc_fn, car_params_data, len(car_params_data))
    lka_state = ffi.new("double[1]")
    lka_state_by_addr = ffi.new("double[1]")
    lka_state_ts = ffi.new("uint16_t[1]")
    lka_state_ts_by_addr = ffi.new("uint16_t[1]")
    can_valid = ffi.new("bool[1]")
    for msg in can_msgs:
      d = msg.as_builder().to_bytes()
      pcp.update_strings([d])
      zmq_msg_data = ffi.new('unsigned char[]', d)
      libcontrolsd_cc.test_canparser_update_string(cp, zmq_msg_data, len(zmq_msg_data), can_valid,
                                                  lka_state, lka_state_by_addr, lka_state_ts, lka_state_ts_by_addr)
    self.assertEqual(pcp.can_valid, can_valid[0])
    self.assertEqual(pcp.vl["EPS_STATUS"]['LKA_STATE'], lka_state[0])
    self.assertEqual(pcp.vl[0x262]['LKA_STATE'], lka_state_by_addr[0])
    self.assertEqual(pcp.ts["EPS_STATUS"]['LKA_STATE'], lka_state_ts[0])
    self.assertEqual(pcp.ts[0x262]['LKA_STATE'], lka_state_ts_by_addr[0])

if __name__ == "__main__":
  unittest.main()
