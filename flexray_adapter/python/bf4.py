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
PROGRESS_FILE = os.path.expanduser("~/.flexray_adapter/bf_progress5.yaml")
PROGRESS3_FILE = os.path.expanduser("~/.flexray_adapter/bf_progress6.yaml")

def load_progress():
  gdNIT = 2
  mkdirs_exists_ok(os.path.dirname(PROGRESS_FILE))
  if os.path.exists(PROGRESS_FILE):
    with open(PROGRESS_FILE, 'r') as f:
      y = yaml.load(f)
      if 'gdNIT' in y:
        gdNIT = y['gdNIT']
  return gdNIT

def load_progress_3():
  gdStaticSlot = 40
  mkdirs_exists_ok(os.path.dirname(PROGRESS3_FILE))
  if os.path.exists(PROGRESS3_FILE):
    with open(PROGRESS3_FILE, 'r') as f:
      y = yaml.load(f)
      if 'gdStaticSlot' in y:
        gdStaticSlot = y['gdStaticSlot']
  return gdStaticSlot

def save_progress3(gdStaticSlot):
  mkdirs_exists_ok(os.path.dirname(PROGRESS3_FILE))
  with open(PROGRESS3_FILE, 'w') as outfile:
    yaml.dump({'gdStaticSlot': gdStaticSlot}, outfile)

def save_progress(gdNIT):
  mkdirs_exists_ok(os.path.dirname(PROGRESS_FILE))
  with open(PROGRESS_FILE, 'w') as outfile:
    yaml.dump({'gdNIT': gdNIT}, outfile)

PROGRESS_GDTSSTRANSMITTER_FILE = os.path.expanduser("~/.flexray_adapter/bf_progress_gdTSSTransmitter2.yaml")

def load_progress_gdTSSTransmitter():
  gdTSSTransmitter = 1
  mkdirs_exists_ok(os.path.dirname(PROGRESS_GDTSSTRANSMITTER_FILE))
  if os.path.exists(PROGRESS_GDTSSTRANSMITTER_FILE):
    with open(PROGRESS3_FILE, 'r') as f:
      y = yaml.load(f)
      if 'gdTSSTransmitter' in y:
        gdTSSTransmitter = y['gdTSSTransmitter']
  return gdTSSTransmitter

def save_progress_gdTSSTransmitter(gdTSSTransmitter):
  mkdirs_exists_ok(os.path.dirname(PROGRESS_GDTSSTRANSMITTER_FILE))
  with open(PROGRESS_GDTSSTRANSMITTER_FILE, 'w') as outfile:
    yaml.dump({'gdTSSTransmitter': gdTSSTransmitter}, outfile)

PROGRESS_GDACTIONPOINTOFFSET_FILE = os.path.expanduser("~/.flexray_adapter/bf_progress_gdActionPointOffset2.yaml")

def load_progress_gdActionPointOffset():
  gdActionPointOffset = 1
  mkdirs_exists_ok(os.path.dirname(PROGRESS_GDACTIONPOINTOFFSET_FILE))
  if os.path.exists(PROGRESS_GDTSSTRANSMITTER_FILE):
    with open(PROGRESS_GDACTIONPOINTOFFSET_FILE, 'r') as f:
      y = yaml.load(f)
      if 'gdActionPointOffset' in y:
        gdActionPointOffset = y['gdActionPointOffset']
  return gdActionPointOffset

def save_progress_gdActionPointOffset(gdActionPointOffset):
  mkdirs_exists_ok(os.path.dirname(PROGRESS_GDACTIONPOINTOFFSET_FILE))
  with open(PROGRESS_GDACTIONPOINTOFFSET_FILE, 'w') as outfile:
    yaml.dump({'gdActionPointOffset': gdActionPointOffset}, outfile)

def factors(n):
  results = set()
  for i in range(1, int(math.sqrt(n)) + 1):
    if n % i == 0:
      results.add(i)
      results.add(int(n / i))
  return results

class BFAlgo2:
  def __init__(self, config):
    # Initial config
    self.config = config
    # Current config used for joining into car flexray network
    self.cur_config = copy.deepcopy(config)
    if (self.config['gdActionPointOffset'] <= self.config['gdMiniSlotActionPointOffset'] or self.config['gNumberOfMinislots'] == 0):
        adActionPointDifference = 0
    else:
        adActionPointDifference = self.config['gdActionPointOffset'] - self.config['gdMiniSlotActionPointOffset']
    # Constraint 18 equation
    self.gMacroPerCycle = self.config['gdStaticSlot'] * self.config['gNumberOfStaticSlots'] + adActionPointDifference + \
                     self.config['gdMinislot'] * self.config['gNumberOfMinislots'] + self.config['gdSymbolWindow'] + \
                     self.config['gdNIT']
    # gdSymbolWindow is in range [0, 162]
    # gdMiniSlotActionPointOffset is in range [1, 31]
    # gdActionPointOffset is in range [1, 63]
    # gdNIT is in range [2, 15978]
    # Start from minimum value of gdNIT
    self.gdNIT = load_progress()
    self.gdSymbolWindow = 0

  # Calculate timing params according to constraint 18
  def caclulate_params(self, log_func):
    # Apparently gNumberOfMinislots of AUDI A4 is not zero.
    if (self.config['gdActionPointOffset'] <= self.config['gdMiniSlotActionPointOffset'] or self.config[
      'gNumberOfMinislots'] == 0):
      adActionPointDifference = 0
    else:
      adActionPointDifference = self.config['gdActionPointOffset'] - self.config['gdMiniSlotActionPointOffset']
    # Constraint 18
    diff = self.gMacroPerCycle - self.config['gdStaticSlot'] * self.config['gNumberOfStaticSlots'] - \
           adActionPointDifference - self.gdSymbolWindow - self.gdNIT
    # diff = gNumberOfMinislots * gdMinislot
    # gNumberOfMinislots is in range [0, 7988]
    # gdMiniSlot is in ange [2, 63]
    for f in factors(diff):
      if 2 <= f <= 63 and 0 < (diff / f) <= 7988:
        return f, int(diff / f)
    log_func('Can not find valid params for diff {}, gdNIT: {}'.format(diff, self.gdNIT))
    return 0, 0

  # Generate next valid config.
  def next(self, log_func):
    # Increase gdNIT until find valid minislot config and symbol window
    while self.gdNIT < 580:
      self.gdSymbolWindow = 580 - self.gdNIT
      if 0 <= self.gdSymbolWindow <= 162:
        gdMinislot, gNumberOfMinislots = self.caclulate_params(log_func)
        if gdMinislot != 0 and gNumberOfMinislots != 0:
          break
      self.gdNIT += 1
    if self.gdNIT >= 580:
      return None
    self.cur_config['gdMinislot'] = gdMinislot
    self.cur_config['gNumberOfMinislots'] = gNumberOfMinislots
    self.cur_config['gdNIT'] = self.gdNIT
    self.cur_config['gdSymbolWindow'] = self.gdSymbolWindow
    # Assume a fixed adOffsetCorrection
    adOffsetCorrection = self.gdNIT - 1
    self.cur_config['gOffsetCorrectionStart'] = self.gMacroPerCycle - adOffsetCorrection
    ok, err = verify_config(self.cur_config)
    if not ok:
      if type(err) == str:
        log_func('gdNIT: {}, invalid config: {}'.format(self.gdNIT, err))
      elif type(err) == tuple:
        log_func('gdNIT: {}, invalid config: {} should be {}'.format(self.gdNIT, err[0], err[1]))
      return None
    self.gdNIT += 1
    return self.cur_config

  def print_config(self):
    r = 'gdNIT: {}'.format(self.cur_config['gdNIT'])
    r += ', gdSymbolWindow: {}'.format(self.cur_config['gdSymbolWindow'])
    r += ', gNumberOfMinislots: {}'.format(self.cur_config['gNumberOfMinislots'])
    r += ', gdMinislot: {}'.format(self.cur_config['gdMinislot'])
    return r


# Brute-force algorthm for finding out the correct length of dynamic segment(gdMinislot * gNumberOfMinislots).
# Based on constraint 18 equation in flexray spec.
# 1) Assume we already have the correct values  of gMacroPerCycle, gdStaticSlot, gNumberOfStaticSlots.
# 2) Assume gdSymbolWindow is zero.
# 3) Try all possible values of gdNIT, calculate the value of gdMinislot * gNumberOfMinislots.
# 4) Generate a valid pair of gdMinislot and gNumberOfMinislots for every possible value of gdNIT
class BFAlgo1:
  def __init__(self, config):
    # Initial config
    self.config = config
    # Current config used for joining into car flexray network
    self.cur_config = copy.deepcopy(config)
    if (self.config['gdActionPointOffset'] <= self.config['gdMiniSlotActionPointOffset'] or self.config['gNumberOfMinislots'] == 0):
        adActionPointDifference = 0
    else:
        adActionPointDifference = self.config['gdActionPointOffset'] - self.config['gdMiniSlotActionPointOffset']
    # Constraint 18 equation
    self.gMacroPerCycle = self.config['gdStaticSlot'] * self.config['gNumberOfStaticSlots'] + adActionPointDifference + \
                     self.config['gdMinislot'] * self.config['gNumberOfMinislots'] + self.config['gdSymbolWindow'] + \
                     self.config['gdNIT']
    # gdSymbolWindow is in range [0, 162]
    # gdMiniSlotActionPointOffset is in range [1, 31]
    # gdActionPointOffset is in range [1, 63]
    # gdNIT is in range [2, 15978]
    # Start from minimum value of gdNIT
    self.gdNIT = load_progress()
    # Assume gdSymbolWindow is zero
    self.gdSymbolWindow = 580 - self.gdNIT

  # Calculate timing params according to constraint 18
  def caclulate_params(self, log_func):
    # Apparently gNumberOfMinislots of AUDI A4 is not zero.
    if (self.config['gdActionPointOffset'] <= self.config['gdMiniSlotActionPointOffset'] or self.config[
      'gNumberOfMinislots'] == 0):
      adActionPointDifference = 0
    else:
      adActionPointDifference = self.config['gdActionPointOffset'] - self.config['gdMiniSlotActionPointOffset']
    # Constraint 18
    diff = self.gMacroPerCycle - self.config['gdStaticSlot'] * self.config['gNumberOfStaticSlots'] - \
           adActionPointDifference - self.gdSymbolWindow - self.gdNIT
    # diff = gNumberOfMinislots * gdMinislot
    # gNumberOfMinislots is in range [0, 7988]
    # gdMiniSlot is in ange [2, 63]
    for f in factors(diff):
      if 2 <= f <= 63 and 0 < (diff / f) <= 7988:
        return f, int(diff / f)
    log_func('Can not find valid params for diff {}, gdNIT: {}'.format(diff, self.gdNIT))
    return 0, 0

  # Generate next valid config.
  def next(self, log_func):
    # Increase gdNIT until find valid minislot config
    while self.gdNIT < 15978:
      gdMinislot, gNumberOfMinislots = self.caclulate_params(log_func)
      if gdMinislot != 0 and gNumberOfMinislots != 0:
          break
      self.gdNIT += 1
    if self.gdNIT >= 15978:
      return None
    self.cur_config['gdMinislot'] = gdMinislot
    self.cur_config['gNumberOfMinislots'] = gNumberOfMinislots
    self.cur_config['gdNIT'] = self.gdNIT
    self.cur_config['gdSymbolWindow'] = self.gdSymbolWindow
    # Assume a fixed adOffsetCorrection
    adOffsetCorrection = self.gdNIT - 1
    self.cur_config['gOffsetCorrectionStart'] = self.gMacroPerCycle - adOffsetCorrection

    self.gdNIT += 1
    return self.cur_config

  def print_config(self):
    r = 'gdNIT: {}'.format(self.cur_config['gdNIT'])
    r += ', gNumberOfMinislots: {}'.format(self.cur_config['gNumberOfMinislots'])
    r += ', gdMinislot: {}'.format(self.cur_config['gdMinislot'])
    return r

# We got good value 53 for gdStaticSlot, also get vSS!BViolation error.
# Lets try bf gdTSSTransmitter([1, 15]) and gdActionPointOffset ([1, 10])
class BFAlgo4:
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
    # [1, 15]
    self.gdTSSTransmitter = load_progress_gdTSSTransmitter()
    # [1, 10]
    self.gdActionPointOffset = load_progress_gdActionPointOffset()
    self.values = []
    for i in range(self.gdTSSTransmitter, 15):
      for j in range(self.gdActionPointOffset, 10):
        self.values.append((i, j))


  # Calculate timing params according to constraint 18
  def caclulate_params(self, log_func):
    # Apparently gNumberOfMinislots of AUDI A4 is not zero.
    if (self.gdActionPointOffset <= self.config['gdMiniSlotActionPointOffset'] or self.config[
      'gNumberOfMinislots'] == 0):
      adActionPointDifference = 0
    else:
      adActionPointDifference = self.gdActionPointOffset - self.config['gdMiniSlotActionPointOffset']
    # Constraint 18
    diff = self.gMacroPerCycle - self.config['gdStaticSlot'] * self.config['gNumberOfStaticSlots'] - \
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
    while len(self.values) > 0:
      self.gdTSSTransmitter, self.gdActionPointOffset = self.values[0]
      self.values = self.values[1:]
      gdMinislot, gNumberOfMinislots = self.caclulate_params(log_func)
      if gdMinislot != 0 and gNumberOfMinislots != 0:
        break
    if gdMinislot == 0 or gNumberOfMinislots == 0:
      return None
    self.cur_config['gdMinislot'] = gdMinislot
    self.cur_config['gNumberOfMinislots'] = gNumberOfMinislots
    self.cur_config['gdTSSTransmitter'] = self.gdTSSTransmitter
    self.cur_config['gdActionPointOffset'] = self.gdActionPointOffset

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
    r = 'gdTSSTransmitter: {}'.format(self.cur_config['gdTSSTransmitter'])
    r += ', gdActionPointOffset: {}'.format(self.cur_config['gdActionPointOffset'])
    r += ', gNumberOfMinislots: {}'.format(self.cur_config['gNumberOfMinislots'])
    r += ', gdMinislot: {}'.format(self.cur_config['gdMinislot'])
    r += ', gdMinislot: {}'.format(self.cur_config['gdStaticSlot'])
    return r

  def save_progress(self):
    save_progress_gdActionPointOffset(self.gdActionPointOffset)
    save_progress_gdTSSTransmitter(self.gdTSSTransmitter)

  @staticmethod
  def get_cur_progress():
    return 'gdTSSTransmitter: {}, gdActionPointOffset: {}'.format(
      load_progress_gdTSSTransmitter(), load_progress_gdActionPointOffset())

if __name__ == '__main__':
  app = QApplication(sys.argv)
  args = get_arg_parser().parse_args(sys.argv[1:])
  ex = BruteForceGUI(args, BFAlgo4, 'bf4.log')
  sys.exit(app.exec())
