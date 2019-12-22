#!/usr/bin/env python3
import unittest
import random
from cffi import FFI

from selfdrive.controls.lib.longcontrol import LongControl
from selfdrive.controls.controlsd_cc import libcontrolsd_cc
from selfdrive.car.toyota.values import CAR
from selfdrive.car.toyota.interface import CarInterface


def compute_gb(accel, speed):
  return float(accel) / 3.0

class TestLongControl(unittest.TestCase):
  def test_cc_equal_py(self):
    CP = CarInterface.get_params(CAR.COROLLAH_TSS2)
    LoC_py = LongControl(CP, compute_gb)
    ffi = FFI()
    car_params_data = ffi.new('unsigned char[]', CP.to_bytes())
    LoC_cc = libcontrolsd_cc.test_longcontrol_init(car_params_data, len(car_params_data))
    final_gas = ffi.new("double[1]")
    final_brake = ffi.new("double[1]")
    v_pid = ffi.new("double[1]")
    pid_p = ffi.new("double[1]")
    pid_i = ffi.new("double[1]")
    pid_f = ffi.new("double[1]")
    for i in range(1024):
      active = (random.randint(0, 2) % 2 == 0)
      v_ego = random.random()
      brake_pressed = (random.randint(0, 2) % 2 == 0)
      standstill = (random.randint(0, 2) % 2 == 0)
      cruise_standstill = (random.randint(0, 2) % 2 == 0)
      v_cruise = random.random()
      v_target = random.random()
      v_target_future = random.random()
      a_target = random.random()
      final_gas_py, final_brake_py = LoC_py.update(active, v_ego, brake_pressed, standstill, cruise_standstill,
                                                   v_cruise, v_target, v_target_future, a_target, CP)
      libcontrolsd_cc.test_longcontrol_update(LoC_cc, car_params_data, len(car_params_data), active, v_ego, brake_pressed,
                                         standstill, cruise_standstill, v_cruise, v_target, v_target_future, a_target,
                                         final_gas, final_brake, v_pid, pid_p, pid_i, pid_f)
      self.assertEqual(final_gas_py, final_gas[0])
      self.assertEqual(final_brake_py, final_brake[0])
      self.assertEqual(LoC_py.v_pid, v_pid[0])
      self.assertEqual(LoC_py.pid.p, pid_p[0])
      self.assertEqual(LoC_py.pid.i, pid_i[0])
      self.assertEqual(LoC_py.pid.f, pid_f[0])


if __name__ == "__main__":
  unittest.main()
