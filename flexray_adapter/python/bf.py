#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os
import sys
import time
import argparse
import copy
from datetime import datetime
from PyQt5.QtWidgets import (
    QWidget, QGroupBox, QPushButton, QMessageBox, QVBoxLayout, QHeaderView, QDoubleSpinBox,
    QSplitter, QHBoxLayout, QLabel, QScrollArea, QListWidget, QFormLayout, QCheckBox, QProgressDialog,
    QDialogButtonBox, QComboBox, QSpinBox, QDialog, QTabWidget, QLineEdit, QGridLayout,
    QTableWidget, QTableWidgetItem, QApplication, QStyle, QListWidgetItem, QRadioButton, QFileDialog)
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer
from PyQt5.QtGui import QIcon
from flexray_config import (default_fr_config, verify_config, verify_config_format)
from flexray_tool import (ReceivePacketsThread, ConnectOrConfigDialog)
from tcp_interface import Connection, ADAPTER_IP_ADDR, ADAPTER_TCP_PORT
from constants import *

class FlexrayParamsGenerator:
  def __init__(self, config):
    # Initial config
    self.config = config
    # Current config
    self.cur_config = copy.deepcopy(config)
    if (self.config['gdActionPointOffset'] <= self.config['gdMiniSlotActionPointOffset'] or self.config['gNumberOfMinislots'] == 0):
        adActionPointDifference = 0
    else:
        adActionPointDifference = self.config['gdActionPointOffset'] - self.config['gdMiniSlotActionPointOffset']
    # Constraint 18:
    gMacroPerCycle = self.config['gdStaticSlot'] * self.config['gNumberOfStaticSlots'] + adActionPointDifference + \
                     self.config['gdMinislot'] * self.config['gNumberOfMinislots'] + self.config['gdSymbolWindow'] + \
                     self.config['gdNIT']
    gdCycle = gMacroPerCycle * self.config['gdMacrotick']
    result.append('gdCycle {} us'.format(gdCycle))

    self.gdSymbolWindow = 0

  def next(self):
    # Try next valid config
    self.cur_config['gdSymbolWindow'] = self.gdSymbolWindow
    return self.cur_config

  def finished(self):
    if self.gdSymbolWindow == 128:
      return True
    self.gdSymbolWindow += 1
    return False

  def print_config(self):
    return 'gdSymbolWindow: {}'.format(self.config['gdSymbolWindow'])

class BruteForceGUI(QWidget):
  def __init__(self, args):
    super().__init__()
    self.args = args
    self.conn = None
    self.elapsed = 0.
    self.timer = QTimer()
    self.timer.timeout.connect(self.on_timer)
    self.recv_packets_thread = None
    self.conn = None
    self.connected = False
    self.cur_config = default_fr_config
    self.tx_frames = self.tx_bytes = self.tx_bps = self.tx_bytes_within_this_second = 0
    self.rx_frames = self.rx_bytes = self.rx_bps = self.rx_bytes_within_this_second = 0
    self.rx_time = self.tx_time = time.time()
    self.join_cluster_timer = QTimer()
    self.join_cluster_timer.timeout.connect(self.on_join_cluster_timeout)
    self.join_cluster_timer.setSingleShot(True)
    self.params_generator = None
    self.existing = False

    self.connect_btn = QPushButton("&Start brute-force...")
    self.connect_btn.setFixedWidth(200)
    self.connect_btn.clicked.connect(self.connect_or_disconnect)
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

    self.log_lv = QListWidget()
    logs_gb_layout = QVBoxLayout()
    logs_gb_layout.addWidget(self.log_lv)
    logs_gb = QGroupBox('Logs')
    logs_gb.setLayout(logs_gb_layout)

    layout = QVBoxLayout()
    layout.addWidget(tool_bar)
    layout.addWidget(stats_gb)
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
      ev.accept()

  def add_log(self, t):
    with open(os.path.expanduser("~/.flexray_adapter/bf.log"), 'a', encoding='utf-8') as f:
      f.write(t + '\n')
    self.log_lv.addItem(t)
    self.log_lv.scrollToBottom()

  def on_connect_timer(self):
    if self.conn.is_connected():
      self.on_connected()
      return
    elif self.elapsed >= 5000.:
      self.on_connect_failed('Connect to {}:{} timeout'.format(self.args.addr, self.args.port))
    else:
      self.elapsed += 100.
      self.timer.singleShot(100., self.on_connect_timer)

  def connect_or_disconnect(self):
    if not self.connected:
      connect_dlg = ConnectOrConfigDialog(self.cur_config, mode='connect')
      r = connect_dlg.exec()
      if r != QDialog.Accepted:
        return
      self.cur_config = connect_dlg.cur_config
      self.params_generator = FlexrayParamsGenerator(self.cur_config)
      for t in connect_dlg.verify_result:
        self.add_log(t)
      self.start_connecting()
    else:
      self.recv_packets_thread.stop()
      self.connect_btn.setEnabled(False)

  def cancel_bruteforce(self):
    self.params_generator = None
    self.cancel_btn.setEnabled(False)
    if self.connected:
      self.recv_packets_thread.stop()
      self.connect_btn.setEnabled(False)
    else:
      self.connect_btn.setEnabled(True)

  def start_connecting(self):
    if not self.params_generator:
      self.connect_btn.setEnabled(True)
      return
    self.status_label_left.setText('Connecting...')
    self.connect_btn.setEnabled(False)
    self.conn = Connection(self.params_generator.next())
    self.conn.connect(addr=args.addr, port=args.port)
    self.elapsed = 0.
    self.timer.singleShot(100., self.on_connect_timer)

  def on_connect_progress(self, progress_text):
    self.add_log(progress_text)

  def on_connected(self):
    self.recv_packets_thread = ReceivePacketsThread(
      self.conn, self.on_frame_received, self.on_recv_pkt_thread_exit, self.on_recv_pkt_thd_exception, self.on_joined_cluster,
      self.on_disconnected_from_cluster, self.on_join_cluster_failed, self.on_fatal_error, self.on_status_data)
    self.connected = True
    self.join_cluster_timer.start(self.args.timeout)
    self.recv_packets_thread.start()
    self.cancel_btn.setEnabled(True)
    self.status_label_left.setText('   Connected   ')
    self.status_label_left.setStyleSheet(self.connected_text_style)
    self.status_label_right.setText('   Joining cluster...   ')

  def on_connect_failed(self, e):
    self.conn.close()
    self.add_log(e)
    self.detail_status.setText('')
    self.status_label_left.setText('   Connect failed.   ')
    self.status_label_left.setStyleSheet(self.disconnected_text_style)
    self.connect_btn.setText('Choose config and Start brute-force')
    self.connect_btn.setEnabled(True)

  def on_recv_pkt_thread_exit(self):
    if self.timer.isActive():
      self.timer.stop()
    self.recv_packets_thread.wait()
    self.recv_packets_thread = None
    self.conn.close()
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
    # Retry
    if True:
      self.start_connecting()
    else:
      self.connect_btn.setText('Choose config and start brute-force')
      self.connect_btn.setEnabled(True)

  def on_recv_pkt_thd_exception(self, e):
    self.add_log(datetime.now().strftime('%H:%M:%S.%f')[:-3] + ' ' + e)

  def on_joined_cluster(self):
    self.status_label_right.setText('   Connected   ')
    self.status_label_right.setStyleSheet(self.connected_text_style)
    self.add_log(datetime.now().strftime('%H:%M:%S.%f')[:-3] + ' Joined into cluster, sniffing on FlexRay bus...')
    if self.join_cluster_timer.isActive():
      self.join_cluster_timer.stop()
    self.add_log(self.params_generator.print_config())
    mb = QMessageBox(QMessageBox.Information, '', "Join cluster succeeded.")
    mb.exec()

  def on_join_cluster_failed(self):
    self.add_log(datetime.now().strftime('%H:%M:%S.%f')[:-3] + ' Join cluster failed, please check the configuration')
    self.status_label_right.setText('   Join cluster failed   ')
    self.status_label_right.setStyleSheet(self.error_text_style)
    if self.join_cluster_timer.isActive():
      self.join_cluster_timer.stop()
    self.connect_or_disconnect()

  def on_disconnected_from_cluster(self):
    self.status_label_right.setText('   Disconnected   ')
    self.status_label_right.setStyleSheet(self.error_text_style)
    self.add_log(datetime.now().strftime('%H:%M:%S.%f')[:-3] + ' Disconnected from cluster, please check the FlexRay cable')

  def on_fatal_error(self):
    self.status_label_right.setText('   Fatal Error   ')
    self.status_label_right.setStyleSheet(self.error_text_style)
    self.add_log(datetime.now().strftime('%H:%M:%S.%f')[:-3] + ' FlexRay fatal error')

  def on_frame_received(self, frame_id, payload_hex, payload_len):
    self.add_log('Rx frame, id: {}, len: {}: '.format(frame_id, payload_len, payload_hex))
    self.rx_frames += 1
    self.rx_bytes += payload_len
    self.rx_bytes_within_this_second += payload_len

  def on_status_data(self, text):
    self.detail_status.setText(text)

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
    self.add_log('Join cluster timeout.')
    self.connect_or_disconnect()

def get_arg_parser():
  parser = argparse.ArgumentParser(
    description="Ground truth generator",
    formatter_class=argparse.ArgumentDefaultsHelpFormatter)

  parser.add_argument("--addr", nargs="?", default=ADAPTER_IP_ADDR,
                      help="IP address of flexray adapter")
  parser.add_argument("--port", nargs="?", default=ADAPTER_TCP_PORT,
                      help="Listen port of flexray adapter")
  parser.add_argument("--timeout", nargs="?", type=float, default=2000.,
                      help="Joining cluster timeout")
  return parser


if __name__ == '__main__':
  app = QApplication(sys.argv)
  args = get_arg_parser().parse_args(sys.argv[1:])
  ex = BruteForceGUI(args)
  sys.exit(app.exec())
