#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os
import sys
import time
import argparse
import copy
import math
import yaml
from datetime import datetime
from PyQt5.QtWidgets import (
    QWidget, QGroupBox, QPushButton, QMessageBox, QVBoxLayout, QSplitter, QHBoxLayout, QLabel, QListWidget,
    QDialog, QGridLayout, QApplication, QStyle, QSpinBox)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QIcon
from flexray_config import (default_fr_config, verify_config, verify_config_format)
from flexray_tool import (mkdirs_exists_ok, ReceivePacketsThread, ConnectOrConfigDialog)
from tcp_interface import Connection, ADAPTER_IP_ADDR, ADAPTER_TCP_PORT
from constants import *
from bf_ui import BruteForceGUI, get_arg_parser

PROGRESS_FILE = os.path.expanduser("~/.flexray_adapter/bf5_progress.yaml")

def load_progress():
  progress = 0
  mkdirs_exists_ok(os.path.dirname(PROGRESS_FILE))
  if os.path.exists(PROGRESS_FILE):
    with open(PROGRESS_FILE, 'r') as f:
      y = yaml.load(f)
      if 'progress' in y:
        progress = y['progress']
  return progress

def save_progress(progress):
  mkdirs_exists_ok(os.path.dirname(PROGRESS_FILE))
  with open(PROGRESS_FILE, 'w') as outfile:
    yaml.dump({
      'progress': progress
    }, outfile)

def factors(n):
  results = set()
  for i in range(1, int(math.sqrt(n)) + 1):
    if n % i == 0:
      results.add(i)
      results.add(int(n / i))
  return results

# We got good value 53 for gdStaticSlot, also get vSS!BViolation error.
# Let's try bf gdStaticSlot[50, 70], gdTSSTransmitter([1, 15]) and gdActionPointOffset ([1, 10])
class BFAlgo5:
  def __init__(self, config):
    # Initial config
    self.config = config
    # Current config that will be used for joining into car flexray network
    self.cur_config = copy.deepcopy(config)
    if (self.config['gdActionPointOffset'] <= self.config['gdMiniSlotActionPointOffset'] or self.config['gNumberOfMinislots'] == 0):
        adActionPointDifference = 0
    else:
        adActionPointDifference = self.config['gdActionPointOffset'] - self.config['gdMiniSlotActionPointOffset']
    # Constraint 18 equation
    self.gMacroPerCycle = self.config['gdStaticSlot'] * self.config['gNumberOfStaticSlots'] + adActionPointDifference + \
                     self.config['gdMinislot'] * self.config['gNumberOfMinislots'] + self.config['gdSymbolWindow'] + \
                     self.config['gdNIT']
    self.values = BFAlgo5.generate_all_values()
    self.progress = load_progress()

  @staticmethod
  def generate_all_values():
    # [1, 15] gdTSSTransmitter
    # [1, 10] gdActionPointOffset
    # [45, 60] gdStaticSlot
    values = []
    for h in range(45, 60 + 1):
      for i in range(1, 15+1):
        for j in range(1, 10 + 1):
          values.append((h, i, j))
    return values

  # Calculate timing params according to constraint 18
  def caclulate_params(self, log_func):
    # Apparently gNumberOfMinislots of AUDI A4 is not zero.
    if (self.gdActionPointOffset <= self.config['gdMiniSlotActionPointOffset'] or self.config[
      'gNumberOfMinislots'] == 0):
      adActionPointDifference = 0
    else:
      adActionPointDifference = self.gdActionPointOffset - self.config['gdMiniSlotActionPointOffset']
    # Constraint 18
    diff = self.gMacroPerCycle - self.gdStaticSlot * self.config['gNumberOfStaticSlots'] - \
           adActionPointDifference - self.config['gdSymbolWindow'] - self.config['gdNIT']
    # diff = gNumberOfMinislots * gdMinislot
    # gNumberOfMinislots is in range [0, 7988]
    # gdMiniSlot is in ange [2, 63]
    for f in factors(diff):
      if 2 <= f <= 63 and 0 < (diff // f) <= 7988:
        return f, diff // f
    log_func('Can not find valid params for diff {}, {}'.format(diff, self.print_config()))
    return 0, 0

  # Generate next valid config.
  def next(self, log_func):
    if len(self.values) <= 0:
      return None
    while self.progress < len(self.values):
      self.gdStaticSlot, self.gdTSSTransmitter, self.gdActionPointOffset = self.values[self.progress]
      self.progress += 1
      gdMinislot, gNumberOfMinislots = self.caclulate_params(log_func)
      if gdMinislot != 0 and gNumberOfMinislots != 0:
        break
    if self.progress >= len(self.values):  # No more values
      return None
    self.cur_config['gdMinislot'] = gdMinislot
    self.cur_config['gNumberOfMinislots'] = gNumberOfMinislots
    self.cur_config['gdTSSTransmitter'] = self.gdTSSTransmitter
    self.cur_config['gdActionPointOffset'] = self.gdActionPointOffset
    self.cur_config['gdStaticSlot'] = self.gdStaticSlot
    # Assume a fixed adOffsetCorrection
    adOffsetCorrection = self.cur_config['gdNIT'] - 1
    self.cur_config['gOffsetCorrectionStart'] = self.gMacroPerCycle - adOffsetCorrection
    while True:
      ok, err = verify_config(self.cur_config)
      if ok:
        break
      if type(err) == str:
        log_func('gdActionPointOffset: {}, gdTSSTransmitter: {}, invalid config: {}'.format(
          self.gdActionPointOffset, self.gdTSSTransmitter, err))
        return None
      elif type(err) == tuple:
        log_func('gdActionPointOffset: {}, gdTSSTransmitter: {}, correct params: {} should be {}'.format(
          self.gdActionPointOffset, self.gdTSSTransmitter, err[0], err[1]))
        self.cur_config[err[0]] = err[1]
    return self.cur_config

  def print_config(self):
    secs = len(self.values) - self.progress
    if secs > 60:
      r = '{} mins {} secs'.format(secs // 60, secs % 60)
    else:
      r = '{} secs'.format(secs)
    r += ', gdTSSTransmitter: {}'.format(self.cur_config['gdTSSTransmitter'])
    r += ', gdActionPointOffset: {}'.format(self.cur_config['gdActionPointOffset'])
    r += ', gdStaticSlot: {}'.format(self.cur_config['gdStaticSlot'])
    return r

  def save_progress(self):
    save_progress(self.progress)

  @staticmethod
  def get_cur_progress():
    values = BFAlgo5.generate_all_values()
    progress = load_progress()
    secs = len(values) - progress
    if secs > 60:
      r = '{} mins {} secs'.format(secs // 60, secs % 60)
    else:
      r = '{} secs'.format(secs)
    return r + ', gdStaticSlot: {}, gdTSSTransmitter: {}, gdActionPointOffset: {}'.format(
      values[progress][0], values[progress][1], values[progress][2])

if __name__ == '__main__':
  app = QApplication(sys.argv)
  args = get_arg_parser().parse_args(sys.argv[1:])
  ex = BruteForceGUI(args, BFAlgo5, 'bf5.log', './test/audi_a4_b9.yml')
  sys.exit(app.exec())
