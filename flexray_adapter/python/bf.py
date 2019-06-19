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

PROGRESS_FILE = os.path.expanduser("~/.flexray_adapter/bf_progress4.yaml")


def load_progress():
  gdNIT = 2
  mkdirs_exists_ok(os.path.dirname(PROGRESS_FILE))
  if os.path.exists(PROGRESS_FILE):
    with open(PROGRESS_FILE, 'r') as f:
      y = yaml.load(f)
      if 'gdNIT' in y:
        gdNIT = y['gdNIT']
  return gdNIT


def save_progress(gdNIT):
  mkdirs_exists_ok(os.path.dirname(PROGRESS_FILE))
  with open(PROGRESS_FILE, 'w') as outfile:
    yaml.dump({'gdNIT': gdNIT}, outfile)


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


class BruteForceGUI(QWidget):
  def __init__(self, args):
    super().__init__()
    self.args = args
    self.conn = None
    self.elapsed = 0.
    self.timer = QTimer()
    self.timer.timeout.connect(self.on_timer)
    self.connect_timer = QTimer()
    self.connect_timer.timeout.connect(self.on_connect_timer)
    self.connect_timer.setSingleShot(True)
    self.recv_packets_thread = None
    self.connected = False
    self.cur_config = default_fr_config
    self.tx_frames = self.tx_bytes = self.tx_bps = self.tx_bytes_within_this_second = 0
    self.rx_frames = self.rx_bytes = self.rx_bps = self.rx_bytes_within_this_second = 0
    self.rx_time = self.tx_time = time.time()
    self.join_cluster_timer = QTimer()
    self.join_cluster_timer.timeout.connect(self.on_join_cluster_timeout)
    self.join_cluster_timer.setSingleShot(True)
    self.bf_algo = None
    self.existing = False

    self.connect_btn = QPushButton("&Start brute-force...")
    self.connect_btn.setFixedWidth(200)
    self.connect_btn.clicked.connect(self.launch_connect_dialogue)
    self.cancel_btn = QPushButton("&Cancel")
    self.cancel_btn.setFixedWidth(200)
    self.cancel_btn.setEnabled(False)
    self.cancel_btn.clicked.connect(self.cancel_bruteforce)
    layout = QHBoxLayout()
    layout.addWidget(self.connect_btn)
    layout.addWidget(self.cancel_btn)
    tool_bar = QWidget()
    tool_bar.setLayout(layout)

    icon_btn_style = "QPushButton { background-color: #FFFFFF; border: 1px solid grey; height: 48;}"
    self.disconnected_text_style = "QPushButton {border: 0px; text-decoration: underline; text-align: center; color : black;}"
    self.connected_text_style = "QPushButton {border: 0px; text-decoration: underline; text-align: center; color : green;}"
    self.error_text_style = "QPushButton {border: 0px; text-decoration: underline; text-align: center; color : red;}"
    layout = QGridLayout()
    btn = QPushButton('My Computer')
    btn.setIcon(QIcon(QApplication.style().standardIcon(QStyle.SP_ComputerIcon)))
    btn.setStyleSheet(icon_btn_style)
    layout.addWidget(btn, 0, 0)
    self.status_label_left = QPushButton('   Not connected   ')
    self.status_label_left.setStyleSheet(self.disconnected_text_style)
    layout.addWidget(self.status_label_left, 0, 1)
    btn = QPushButton('FlexRay Adapter')
    btn.setStyleSheet(icon_btn_style)
    layout.addWidget(btn, 0, 2)
    self.status_label_right = QPushButton('   Not connected   ')
    self.status_label_right.setStyleSheet(self.disconnected_text_style)
    layout.addWidget(self.status_label_right, 0, 3)
    btn = QPushButton('FlexRay Network')
    btn.setStyleSheet(icon_btn_style)
    layout.addWidget(btn, 0, 4)
    self.detail_status = QLabel()
    self.detail_status.setWordWrap(True)
    layout.addWidget(self.detail_status, 1, 0)
    self.statistics_label = QLabel()
    self.statistics_label.setWordWrap(True)
    layout.addWidget(self.statistics_label, 1, 2)
    stats_gb = QGroupBox('Status')
    stats_gb.setLayout(layout)

    self.monitored_slots = [8, 24, 31, 45]
    layout = QHBoxLayout()
    for i in range(4):
      w = QSpinBox()
      w.setMinimum(0)
      w.setMaximum(127)
      w.setValue(self.monitored_slots[i])
      w.valueChanged.connect(lambda val,i=i: self.set_monitored_slots(i, val))
      layout.addWidget(w)
    btn = QPushButton("&Upate")
    btn.clicked.connect(self.send_monitored_slots)
    layout.addWidget(btn)
    monitored_slots_form = QGroupBox('Monitored Slots')
    monitored_slots_form.setLayout(layout)

    self.log_lv = QListWidget()
    logs_gb_layout = QVBoxLayout()
    logs_gb_layout.addWidget(self.log_lv)
    logs_gb = QGroupBox('Logs')
    logs_gb.setLayout(logs_gb_layout)

    layout = QVBoxLayout()
    self.progress_label = QLabel()
    self.progress_label.setText('gdNIT: {}, gdSymbolWindow: {}'.format(load_progress(), 580 - load_progress()))
    layout.addWidget(self.progress_label)
    progress_gb = QGroupBox('Progress')
    progress_gb.setLayout(layout)

    layout = QVBoxLayout()
    layout.addWidget(tool_bar)
    layout.addWidget(stats_gb)
    layout.addWidget(progress_gb)
    layout.addWidget(monitored_slots_form)
    w = QWidget()
    w.setLayout(layout)
    splitter = QSplitter(Qt.Vertical, self)
    splitter.addWidget(w)
    splitter.addWidget(logs_gb)
    layout = QVBoxLayout()
    layout.addWidget(splitter)
    self.setLayout(layout)
    layout.setContentsMargins(0, 0, 0, 0)
    self.setGeometry(200, 200, 800, 450)
    self.setWindowTitle('FlexRay Brute-force Tool')
    self.show()

  def closeEvent(self, ev):
    if self.connected:
      self.recv_packets_thread.stop()
      self.existing = True
      ev.ignore()
    else:
      if self.connect_timer.isActive():
        self.connect_timer.stop()
      ev.accept()

  def set_monitored_slots(self, i, val):
    self.monitored_slots[i] = val

  def send_monitored_slots(self):
    self.add_log('Set monitored slots: {}, {}, {}, {}'.format(*self.monitored_slots))
    if self.conn:
      self.conn.set_monitored_slots(self.monitored_slots)

  def add_log(self, text):
    t = datetime.now().strftime('%H:%M:%S.%f')[:-3]
    with open(os.path.expanduser("~/.flexray_adapter/bf.log"), 'a', encoding='utf-8') as f:
      f.write(t + text + '\n')
    self.log_lv.addItem(t + text)
    self.log_lv.scrollToBottom()

  def on_connect_timer(self):
    if self.conn.is_connected():
      self.connect_timer.stop()
      self.on_connected()
      return
    elif self.elapsed >= 5000.:
      self.connect_timer.stop()
      self.on_connect_failed('Connect to {}:{} timeout'.format(self.args.addr, self.args.port))
    else:
      self.elapsed += 100.
      self.connect_timer.start(100.)

  # Step 1, Choose a flexray config file
  def launch_connect_dialogue(self):
    if not self.connected:
      connect_dlg = ConnectOrConfigDialog(self.cur_config, mode='connect')
      r = connect_dlg.exec()
      if r != QDialog.Accepted:
        return
      self.cur_config = connect_dlg.cur_config
      self.add_log('Config file loaded: {}'.format(connect_dlg.cur_config_file))
      #self.bf_algo = BFAlgo1(self.cur_config)
      self.bf_algo = BFAlgo2(self.cur_config)
      for t in connect_dlg.verify_result:
        self.add_log(t)
      self.start_connecting()

  def disconnect(self):
    if self.connected:
      self.recv_packets_thread.stop()
      self.connect_btn.setEnabled(False)

  def cancel_bruteforce(self):
    self.bf_algo = None
    self.cancel_btn.setEnabled(False)
    if self.connected:
      self.recv_packets_thread.stop()
      self.connect_btn.setEnabled(False)
    else:
      self.connect_btn.setEnabled(True)

  # Step 2, Generate a config by call bf_algo.next, then connect to devkit.
  def start_connecting(self):
    if not self.bf_algo:
      self.connect_btn.setEnabled(True)
      return
    config = self.bf_algo.next(self.add_log)
    if not config:
      self.add_log('Brute-force finished.')
      self.connect_btn.setEnabled(True)
      self.cancel_btn.setEnabled(False)
      return
    self.progress_label.setText(
      'gdNIT: {}, gdSymbolWindow: {}'.format(self.bf_algo.cur_config['gdNIT'], self.bf_algo.cur_config['gdSymbolWindow']))
    self.add_log('Begin test: {}'.format(self.bf_algo.print_config()))
    self.status_label_left.setText('Connecting...')
    self.connect_btn.setEnabled(False)
    self.conn = Connection(config)
    self.conn.connect(addr=args.addr, port=args.port)
    self.elapsed = 0.
    self.connect_timer.start(100.)

  def on_connect_progress(self, progress_text):
    self.add_log(progress_text)

  # Step 3, after tcp connection established and config sent to devkit, wait join cluster result.
  def on_connected(self):
    self.recv_packets_thread = ReceivePacketsThread(
      self.conn, self.on_frame_received, self.on_recv_pkt_thread_exit, self.on_recv_pkt_thd_exception, self.on_joined_cluster,
      self.on_disconnected_from_cluster, self.on_join_cluster_failed, self.on_fatal_error, self.on_status_data)
    self.connected = True
    # Sometimes joining cluster will just timeout without any error, so we schedule a timer for this situation.
    self.join_cluster_timer.start(self.args.timeout)
    self.recv_packets_thread.start()
    self.cancel_btn.setEnabled(True)
    self.status_label_left.setText('   Connected   ')
    self.status_label_left.setStyleSheet(self.connected_text_style)
    self.status_label_right.setText('   Joining cluster...   ')
    self.send_monitored_slots()

  # Handle TCP connection error
  def on_connect_failed(self, e):
    self.conn.close()
    self.conn = None
    self.add_log(e)
    self.detail_status.setText('')
    self.status_label_left.setText('   Connect failed.   ')
    self.status_label_left.setStyleSheet(self.disconnected_text_style)
    self.connect_btn.setText('Choose config and Start brute-force')
    self.connect_btn.setEnabled(True)

  # After joining cluster failed or timeout, recv packet thread will exit and this function will be called.
  def on_recv_pkt_thread_exit(self):
    if self.timer.isActive():
      self.timer.stop()
    self.recv_packets_thread.wait()
    self.recv_packets_thread = None
    self.conn.close()
    self.conn = None
    self.detail_status.setText('')
    self.status_label_left.setText('   Disconnected.   ')
    self.status_label_left.setStyleSheet(self.disconnected_text_style)
    self.status_label_right.setText('   Disconnected.   ')
    self.status_label_right.setStyleSheet(self.disconnected_text_style)
    self.rx_frames = self.rx_bytes = self.rx_bps = self.rx_bytes_within_this_second = 0
    self.tx_frames = self.tx_bytes = self.tx_bps = self.tx_bytes_within_this_second = 0
    self.connected = False
    if self.existing:
      self.close()  # closeEVent will be called again
      return
    save_progress(self.bf_algo.cur_config['gdNIT'])
    # Retry joining cluster with another set of params
    self.start_connecting()

  def on_recv_pkt_thd_exception(self, e):
    self.add_log(datetime.now().strftime('%H:%M:%S.%f')[:-3] + ' ' + e)

  # Joining cluster succeeded
  def on_joined_cluster(self):
    self.status_label_right.setText('   Connected   ')
    self.status_label_right.setStyleSheet(self.connected_text_style)
    self.add_log(' Joined into cluster, sniffing on FlexRay bus...')
    if self.join_cluster_timer.isActive():
      self.join_cluster_timer.stop()
    self.add_log(self.bf_algo.print_config())
    self.timer.start(1000)
    mb = QMessageBox(QMessageBox.Information, '', "Join cluster succeeded.")
    mb.exec()

  # Joining cluster failed.
  def on_join_cluster_failed(self):
    self.add_log(datetime.now().strftime('%H:%M:%S.%f')[:-3] + ' Join cluster failed, please check the configuration')
    self.status_label_right.setText('   Join cluster failed   ')
    self.status_label_right.setStyleSheet(self.error_text_style)
    if self.join_cluster_timer.isActive():
      self.join_cluster_timer.stop()
    self.disconnect()

  def on_disconnected_from_cluster(self):
    self.status_label_right.setText('   Disconnected   ')
    self.status_label_right.setStyleSheet(self.error_text_style)
    self.add_log(datetime.now().strftime('%H:%M:%S.%f')[:-3] + ' Disconnected from cluster, please check the FlexRay cable')
    if self.time.isActive():
      self.timer.stop()

  def on_fatal_error(self, err):
    self.status_label_right.setText('   Fatal Error {}   '.format(err))
    self.status_label_right.setStyleSheet(self.error_text_style)
    self.add_log(datetime.now().strftime('%H:%M:%S.%f')[:-3] + ' FlexRay fatal error, code: {}'.format(err))

  def on_frame_received(self, frame_id, payload_hex, payload_len):
    self.add_log('Rx frame, id: {}, len: {}: '.format(frame_id, payload_len, payload_hex))
    self.rx_frames += 1
    self.rx_bytes += payload_len
    self.rx_bytes_within_this_second += payload_len

  def on_status_data(self, text, ssr0, ssr1, ssr2, ssr3, ssr4, ssr5, ssr6, ssr7):
    self.detail_status.setText(text)
    for t in text.split('\n'):
      self.add_log('gdNIT: {}, {}'.format(self.bf_algo.cur_config['gdNIT'], t))
    r = []
    r.append('Slot {} status, even cycle'.format(self.monitored_slots[0]))
    ReceivePacketsThread.parse_slot_status(ssr0, r)
    r.append('Slot {} status, odd cycle'.format(self.monitored_slots[0]))
    ReceivePacketsThread.parse_slot_status(ssr1, r)
    r.append('Slot {} status, even cycle'.format(self.monitored_slots[1]))
    ReceivePacketsThread.parse_slot_status(ssr2, r)
    r.append('Slot {} status, odd cycle'.format(self.monitored_slots[1]))
    ReceivePacketsThread.parse_slot_status(ssr3, r)
    r.append('Slot {} status, even cycle'.format(self.monitored_slots[2]))
    ReceivePacketsThread.parse_slot_status(ssr4, r)
    r.append('Slot {} status, odd cycle'.format(self.monitored_slots[2]))
    ReceivePacketsThread.parse_slot_status(ssr5, r)
    r.append('Slot {} status, even cycle'.format(self.monitored_slots[3]))
    ReceivePacketsThread.parse_slot_status(ssr6, r)
    r.append('Slot {} status, odd cycle'.format(self.monitored_slots[3]))
    ReceivePacketsThread.parse_slot_status(ssr7, r)
    for t in r:
      self.add_log(t)

  def update_statistics_label(self):
    if self.tx_bps > 1000:
      tx_speed_text = '{}K'.format(int(self.tx_bps / 1000.0))
    else:
      tx_speed_text = str(self.tx_bps)
    if self.rx_bps > 1000:
      rx_speed_text = '{}K'.format(int(self.rx_bps / 1000.0))
    else:
      rx_speed_text = str(self.tx_bps)
    self.statistics_label.setText(
      'Tx Frames: {}\nTx bytes: {}\nTx Speed: {}bps\nRx frames: {}\nRx bytes: {}\nRx Speed: {}bps'.format(
          self.tx_frames, self.tx_bytes, tx_speed_text, self.rx_frames, self.rx_bytes, rx_speed_text))

  def on_timer(self):
    self.tx_bps = self.tx_bytes_within_this_second * 8
    self.tx_time = time.time()
    self.tx_bytes_within_this_second = 0
    self.rx_bps = self.rx_bytes_within_this_second * 8
    self.rx_time = time.time()
    self.rx_bytes_within_this_second = 0
    self.update_statistics_label()

  def on_join_cluster_timeout(self):
    self.add_log('gdNIT: {}, Join cluster timeout.'.format(self.bf_algo.cur_config['gdNIT']))
    self.disconnect()

def get_arg_parser():
  parser = argparse.ArgumentParser(
    description="Ground truth generator",
    formatter_class=argparse.ArgumentDefaultsHelpFormatter)

  parser.add_argument("--addr", nargs="?", default=ADAPTER_IP_ADDR,
                      help="IP address of flexray adapter.")
  parser.add_argument("--port", nargs="?", default=ADAPTER_TCP_PORT,
                      help="Listen port of flexray adapter.")
  parser.add_argument("--timeout", nargs="?", type=float, default=5 * 1000.,
                      help="Wait timeout for joining cluster.")
  return parser


if __name__ == '__main__':
  app = QApplication(sys.argv)
  args = get_arg_parser().parse_args(sys.argv[1:])
  ex = BruteForceGUI(args)
  sys.exit(app.exec())
