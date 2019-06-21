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

# Bruteforce gdStaticSlot
class BFAlgo3:
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
    # gdStaticSlot is within [40, 70] from waveform
    # gdActionPointOffset is 1
    self.gdStaticSlot = load_progress_3()

  # Calculate timing params according to constraint 18
  def caclulate_params(self, log_func):
    # Apparently gNumberOfMinislots of AUDI A4 is not zero.
    if (self.config['gdActionPointOffset'] <= self.config['gdMiniSlotActionPointOffset'] or self.config[
      'gNumberOfMinislots'] == 0):
      adActionPointDifference = 0
    else:
      adActionPointDifference = self.config['gdActionPointOffset'] - self.config['gdMiniSlotActionPointOffset']
    # Constraint 18
    diff = self.gMacroPerCycle - self.gdStaticSlot * self.config['gNumberOfStaticSlots'] - \
           adActionPointDifference - self.config['gdSymbolWindow'] - self.config['gdNIT']
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
    # Increase gdStaticSlot until find valid minislot config and symbol window
    while self.gdStaticSlot < 70:
      gdMinislot, gNumberOfMinislots = self.caclulate_params(log_func)
      if gdMinislot != 0 and gNumberOfMinislots != 0:
        break
      self.gdStaticSlot += 1
    if self.gdStaticSlot >= 70:
      return None
    self.cur_config['gdMinislot'] = gdMinislot
    self.cur_config['gNumberOfMinislots'] = gNumberOfMinislots
    self.cur_config['gdStaticSlot'] = self.gdStaticSlot
    # Assume a fixed adOffsetCorrection
    adOffsetCorrection = self.cur_config['gdNIT'] - 1
    self.cur_config['gOffsetCorrectionStart'] = self.gMacroPerCycle - adOffsetCorrection
    ok, err = verify_config(self.cur_config)
    if not ok:
      if type(err) == str:
        log_func('gdNIT: {}, invalid config: {}'.format(self.gdNIT, err))
      elif type(err) == tuple:
        log_func('gdNIT: {}, invalid config: {} should be {}'.format(self.gdNIT, err[0], err[1]))
      return None
    self.gdStaticSlot += 1
    return self.cur_config

  def print_config(self):
    r = 'gdStaticSlot: {}'.format(self.cur_config['gdStaticSlot'])
    r += ', gNumberOfMinislots: {}'.format(self.cur_config['gNumberOfMinislots'])
    r += ', gdMinislot: {}'.format(self.cur_config['gdMinislot'])
    r += ', gdTSSTransmitter: {}'.format(self.cur_config['gdTSSTransmitter'])
    r += ', gdActionPointOffset: {}'.format(self.cur_config['gdActionPointOffset'])
    return r

  def save_progress(self):
    save_progress3(self.cur_config['gdStaticSlot'])

  @staticmethod
  def get_cur_progress():
    return 'gdStaticSlot: {}, '.format(
      load_progress_3())

if __name__ == '__main__':
  app = QApplication(sys.argv)
  args = get_arg_parser().parse_args(sys.argv[1:])
  ex = BruteForceGUI(args, BFAlgo3, 'bf3.log')
  sys.exit(app.exec())
