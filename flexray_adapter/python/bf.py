#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os
import glob
import sys
import struct
import time
import threading
import yaml
from datetime import datetime
from PyQt5.QtWidgets import (
    QWidget, QGroupBox, QPushButton, QMessageBox, QVBoxLayout, QHeaderView, QDoubleSpinBox,
    QSplitter, QHBoxLayout, QLabel, QScrollArea, QListWidget, QFormLayout, QCheckBox, QProgressDialog,
    QDialogButtonBox, QComboBox, QSpinBox, QDialog, QTabWidget, QLineEdit, QGridLayout,
    QTableWidget, QTableWidgetItem, QApplication, QStyle, QListWidgetItem, QRadioButton, QFileDialog)
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer
from PyQt5.QtGui import QIcon
from tcp_interface import Connection
from flexray_config import (default_fr_config, verify_config, verify_config_format)
from flexray_tool import (ReceivePacketsThread, ConnectOrConfigDialog)
from tcp_interface import Connection
from constants import *

class BruteForceGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.conn = None
        self.elapsed = 0.
        self.timer = QTimer()
        self.timer.timeout.connect(self.on_timer)
        self.recv_packets_thread = None
        self.conn = None
        self.cur_config = default_fr_config
        self.send_frame_dlg = None
        self.tx_frames = self.tx_bytes = self.tx_bps = self.tx_bytes_within_this_second = 0
        self.rx_frames = self.rx_bytes = self.rx_bps = self.rx_bytes_within_this_second = 0
        self.rx_time = self.tx_time = time.time()
        header = ['Time', 'Frame ID', 'Length', 'HEX']
        self.frame_table = QTableWidget()
        self.frame_table.setColumnCount(len(header))
        self.frame_table.setHorizontalHeaderLabels(header)
        self.frame_table.setMinimumSize(400, 300)
        self.frame_table.setSortingEnabled(True)

        clear_rx_frames_btn = QPushButton("&Remove All")
        clear_rx_frames_btn.clicked.connect(self.clear_rx_frame_table)
        clear_rx_frames_btn.setMaximumWidth(100)
        clear_rx_frames_btn.setIcon(QIcon(QApplication.style().standardIcon(QStyle.SP_TrashIcon)))
        layout = QVBoxLayout()
        layout.addWidget(clear_rx_frames_btn)
        layout.addWidget(self.frame_table)
        self.rx_frames_gb = QGroupBox('Rx Frames')
        self.rx_frames_gb.setLayout(layout)

        self.connect_btn = QPushButton("Choose config and Start brute-force")
        self.connect_btn.setFixedWidth(400)
        self.connect_btn.clicked.connect(self.connect_or_disconnect)
        layout = QHBoxLayout()
        layout.addWidget(self.connect_btn)
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
        bottom_vbox_layout = QVBoxLayout()
        bottom_vbox_layout.addWidget(self.log_lv)

        bottom_group_box = QGroupBox('Logs')
        bottom_group_box.setLayout(bottom_vbox_layout)

        layout = QVBoxLayout()
        layout.addWidget(tool_bar)
        layout.addWidget(self.rx_frames_gb)
        layout.addWidget(stats_gb)
        w = QWidget()
        w.setLayout(layout)
        splitter = QSplitter(Qt.Vertical, self)
        splitter.addWidget(w)
        splitter.addWidget(bottom_group_box)
        layout = QVBoxLayout()
        layout.addWidget(splitter)
        self.setLayout(layout)
        layout.setContentsMargins(0, 0, 0, 0)
        self.setGeometry(200, 200, 800, 450)
        self.setWindowTitle('FlexRay Brute-force Tool')
        self.show()

    def add_log(self, t):
      with open(os.path.expanduser("~/.flexray_adapter/bf.log"), 'a', encoding='utf-8') as f:
        f.write(t)
      self.log_lv.addItem(t)
      self.log_lv.scrollToBottom()

    def on_connect_timer(self):
      if self.conn.is_connected():
        self.on_connected()
        return
      elif self.elapsed >= 5000.:
        self.on_connect_failed('Connect timeout')
      else:
        self.elapsed += 100.
        self.timer.singleShot(100., self.on_connect_timer)

    def connect_or_disconnect(self):
        if not self.recv_packets_thread:
            connect_dlg = ConnectOrConfigDialog(self.cur_config, mode='connect')
            r = connect_dlg.exec()
            self.cur_config = connect_dlg.cur_config
            if r != QDialog.Accepted:
                return
            for t in connect_dlg.verify_result:
                self.add_log(t)
            self.status_label_left.setText('Connecting...')
            self.connect_btn.setEnabled(False)

            self.conn = Connection(self.cur_config)
            self.conn.connect()
            self.elapsed = 0.
            self.timer.singleShot(100., self.on_connect_timer)
        else:
            self.recv_packets_thread.stop()
            self.connect_btn.setEnabled(False)

    def on_connect_progress(self, progress_text):
        self.add_log(progress_text)

    def on_connected(self):
        self.recv_packets_thread = ReceivePacketsThread(
            self.conn, self.on_frame_received, self.on_sock_disconnect, self.on_recv_pkt_thd_exception, self.on_joined_cluster,
            self.on_disconnected_from_cluster, self.on_join_cluster_failed, self.on_fatal_error, self.on_status_data)
        self.recv_packets_thread.start()
        self.connect_btn.setText('Disconnect')
        self.connect_btn.setEnabled(True)
        self.status_label_left.setText('   Connected   ')
        self.status_label_left.setStyleSheet(self.connected_text_style)
        self.status_label_right.setText('   Joining cluster...   ')

    def on_connect_failed(self, e):
        self.add_log(e)
        self.detail_status.setText('')
        self.status_label_left.setText('   Connect failed.   ')
        self.status_label_left.setStyleSheet(self.disconnected_text_style)
        self.connect_btn.setText('Join into FlexRay network')
        self.connect_btn.setEnabled(True)

    def on_sock_disconnect(self):
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
        self.send_frame_btn.setEnabled(False)
        self.rx_frames = self.rx_bytes = self.rx_bps = self.rx_bytes_within_this_second = 0
        self.tx_frames = self.tx_bytes = self.tx_bps = self.tx_bytes_within_this_second = 0
        self.connect_btn.setText('Choose config and start brute-force')
        self.connect_btn.setEnabled(True)

    def on_recv_pkt_thd_exception(self, e):
        self.add_log(datetime.now().strftime('%H:%M:%S.%f')[:-3] + ' '+ e)

    def on_joined_cluster(self):
        self.status_label_right.setText('   Connected   ')
        self.status_label_right.setStyleSheet(self.connected_text_style)
        self.timer.start(1000)
        self.add_log(datetime.now().strftime('%H:%M:%S.%f')[:-3] + ' Joined into cluster, sniffing on FlexRay bus...')
        self.send_frame_btn.setEnabled(True)

    def on_join_cluster_failed(self):
        self.status_label_right.setText('   Join cluster failed   ')
        self.status_label_right.setStyleSheet(self.error_text_style)
        self.add_log(datetime.now().strftime('%H:%M:%S.%f')[:-3] + 'Join cluster failed, please check the configuration')

    def on_disconnected_from_cluster(self):
        self.status_label_right.setText('   Disconnected   ')
        self.status_label_right.setStyleSheet(self.error_text_style)
        self.add_log(datetime.now().strftime('%H:%M:%S.%f')[:-3] + ' Disconnected from cluster, please check the FlexRay cable')

    def on_fatal_error(self):
        self.status_label_right.setText('   Fatal Error   ')
        self.status_label_right.setStyleSheet(self.error_text_style)
        self.add_log(datetime.now().strftime('%H:%M:%S.%f')[:-3] + ' FlexRay fatal error')

    def on_frame_received(self, frame_id, payload_hex, payload_len):
        row_count = self.frame_table.rowCount()
        self.frame_table.setRowCount(row_count + 1)
        self.frame_table.setItem(row_count, 0, QTableWidgetItem(datetime.now().strftime('%H:%M:%S.%f')[:-3]))
        self.frame_table.setItem(row_count, 1, QTableWidgetItem(frame_id))
        self.frame_table.setItem(row_count, 2, QTableWidgetItem(str(payload_len)))
        self.frame_table.setItem(row_count, 3, QTableWidgetItem(payload_hex))
        self.rx_frames += 1
        self.rx_frames_gb.setTitle('Rx Frames ({})'.format(self.rx_frames))
        self.rx_bytes += payload_len
        self.rx_bytes_within_this_second += payload_len

    def on_status_data(self, text):
        self.detail_status.setText(text)

    def clear_rx_frame_table(self):
        self.frame_table.clearContents()
        self.frame_table.setRowCount(0)
        self.rx_frames = 0
        self.rx_bytes = 0
        self.rx_frames_gb.setTitle('Rx Frames ({})'.format(self.rx_frames))

    def on_sent_frame(self, payload_len):
        self.tx_frames += 1
        self.tx_bytes += payload_len
        self.tx_bytes_within_this_second += payload_len

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


if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = BruteForceGUI()
    sys.exit(app.exec())
