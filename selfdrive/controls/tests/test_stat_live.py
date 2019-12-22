#!/usr/bin/env python3
import unittest
import random
from cffi import FFI

from common.stat_live import RunningStatFilter
from selfdrive.controls.controlsd_cc import libcontrolsd_cc


class TestStatLive(unittest.TestCase):
  def test_cc_equal_py(self):
    max_trackable = 3600
    rsf_py = RunningStatFilter(max_trackable=max_trackable)
    rsf_cc = libcontrolsd_cc.test_running_stat_filter_init(max_trackable)
    ffi = FFI()
    out = ffi.new("double[6]")
    for i in range(1024):
      v = random.random()
      rsf_py.push_and_update(v)
      libcontrolsd_cc.test_running_stat_filter_push_and_update(rsf_cc, v)
      libcontrolsd_cc.test_running_stat_filter_query(rsf_cc, out)
      self.assertEqual(out[0], rsf_py.filtered_stat.n)
      self.assertEqual(out[1], rsf_py.filtered_stat.M)
      self.assertEqual(out[2], rsf_py.filtered_stat.S)
      self.assertEqual(out[3], rsf_py.filtered_stat.M_last)
      self.assertEqual(out[4], rsf_py.filtered_stat.S_last)
      self.assertEqual(out[5], rsf_py.filtered_stat.n)


if __name__ == "__main__":
  unittest.main()
