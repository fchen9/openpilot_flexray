#!/usr/bin/env python3
import unittest
import random
from cffi import FFI

import selfdrive.messaging as messaging
from selfdrive.controls.lib.latcontrol_pid import LatControlPID
from selfdrive.controls.controlsd_cc import libcontrolsd_cc
from selfdrive.car.toyota.values import CAR
from selfdrive.car.toyota.interface import CarInterface

class TestLatControlPID(unittest.TestCase):
  def test_cc_equal_py(self):
    CP = CarInterface.get_params(CAR.COROLLAH_TSS2)
    LaC_py = LatControlPID(CP)
    ffi = FFI()
    car_params_bytes = CP.to_bytes()
    car_params_data = ffi.new('unsigned char[]', car_params_bytes)
    LaC_cc = libcontrolsd_cc.test_latcontrol_pid_init(car_params_data, len(car_params_bytes))
    output_steer_v = ffi.new("double[1]")
    output_steering_angle = ffi.new("double[1]")
    pid_active = ffi.new("bool[1]")
    pid_steer_angle = ffi.new("float[1]")
    pid_steer_rate = ffi.new("float[1]")
    for i in range(1024):
      active = (random.randint(0, 2) % 2 == 0)
      v_ego = random.random()
      angle_steers = random.random()
      angle_steers_rate = random.random()
      eps_torque = random.random()
      steer_override = (random.randint(0, 2) % 2 == 0)

      path_plan = messaging.new_message()
      path_plan.init('pathPlan')
      path_plan.valid = True
      path_plan.pathPlan.laneWidth = random.random()
      path_plan.pathPlan.dPoly = [random.random(), random.random(), random.random(), random.random()]
      path_plan.pathPlan.lPoly = [random.random(), random.random(), random.random(), random.random()]
      path_plan.pathPlan.lProb = random.random()
      path_plan.pathPlan.rPoly = [random.random(), random.random(), random.random(), random.random()]
      path_plan.pathPlan.rProb = random.random()

      path_plan.pathPlan.angleSteers = random.random()
      path_plan.pathPlan.rateSteers = random.random()
      path_plan.pathPlan.angleOffset = random.random()
      path_plan.pathPlan.mpcSolutionValid = (random.randint(0, 2) % 2 == 0)
      path_plan.pathPlan.paramsValid = (random.randint(0, 2) % 2 == 0)
      path_plan.pathPlan.sensorValid = (random.randint(0, 2) % 2 == 0)
      path_plan.pathPlan.posenetValid = (random.randint(0, 2) % 2 == 0)

      output_steer_v_py, output_steering_angle_py, pid_log_py = \
        LaC_py.update(active, v_ego, angle_steers, angle_steers_rate, eps_torque, steer_override, CP, path_plan.pathPlan)
      path_plan_bytes = path_plan.to_bytes()
      path_plan_data = ffi.new('unsigned char[]', path_plan_bytes)
      libcontrolsd_cc.test_latcontrol_pid_update( LaC_cc, car_params_data, len(car_params_bytes), path_plan_data,
                                                  len(path_plan_bytes), active, v_ego, angle_steers, angle_steers_rate,
                                                  eps_torque, steer_override, output_steer_v, output_steering_angle,
                                                  pid_active, pid_steer_angle, pid_steer_rate)
      self.assertEqual(output_steer_v_py, output_steer_v[0])
      self.assertEqual(output_steering_angle_py, output_steering_angle[0])
      self.assertEqual(pid_log_py.active, pid_active[0])
      self.assertEqual(pid_log_py.steerAngle, pid_steer_angle[0])
      self.assertEqual(pid_log_py.steerRate, pid_steer_rate[0])

if __name__ == "__main__":
  unittest.main()
