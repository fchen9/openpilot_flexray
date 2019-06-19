import socket
import struct
import threading
import time
from select import select
from flexray_config import (config_to_c_struct, map_frame_id_to_tx_msg_buf_idx)
from constants import *

ADAPTER_IP_ADDR = '192.168.5.10'
ADAPTER_TCP_PORT = 3888

class HealthThd(threading.Thread):
    def __init__(self, conn):
        super(HealthThd, self).__init__()
        self._stop_event = threading.Event()
        self._conn = conn

    def stop(self):
        self._stop_event.set()

    def stopped(self):
        return self._stop_event.is_set()

    def run(self):
        try:
            while not self.stopped():
                self._conn.send_packet(PACKET_TYPE_HEALTH, b'')
                time.sleep(1)
        finally:
            pass


class Connection:
  def __init__(self, fr_config):
    self.sock = None
    self.fr_config = fr_config
    self.frame_id_to_tx_msg_buf_idx = map_frame_id_to_tx_msg_buf_idx(fr_config)
    self.health_thd = None
    self.send_lock = threading.Lock()
    self.read_buf = b''

  def connect(self, addr=ADAPTER_IP_ADDR, port=ADAPTER_TCP_PORT):
    self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    self.sock.setblocking(0)
    try:
      self.sock.connect((addr, port))
    except BlockingIOError:
      pass

  def is_connected(self):
    _, writables, _ = select([], [self.sock], [], 0.)
    if len(writables) <= 0:
      return False
    self.start_driver()
    return True

  def start_driver(self):
    self.send_packet(PACKET_TYPE_START_DRIVER, config_to_c_struct(self.fr_config))
    self.health_thd = HealthThd(self)
    self.health_thd.start()

  def set_monitored_slots(self, slot_ids):
    self.send_packet(PACKET_TYPE_MONIOR_SLOTS, struct.pack('>4H', *slot_ids))

  def close(self):
    if self.health_thd:
      self.health_thd.stop()
      self.health_thd.join()
    if self.sock:
      self.sock.close()
    self.read_buf = b''

  def send_all(self, data):
    amount_sent = 0
    while amount_sent < len(data):
      _, writables, _ = select([], [self.sock], [], 1)
      if len(writables) > 0:
          amount_sent += self.sock.send(data[amount_sent:])

  def receive_packet(self, on_packet_received, on_peer_shutdown):
    readables, _, _ = select([self.sock], [], [], 0.5)
    if len(readables) > 0:
      if len(self.read_buf) < SIZEOF_PACKET_HEADER:
        required_len = SIZEOF_PACKET_HEADER - len(self.read_buf)
      else:
        header = struct.unpack('>HH', self.read_buf[0:SIZEOF_PACKET_HEADER])
        if header[0] < SIZEOF_PACKET_HEADER:
          raise RuntimeError("Invalid packet length: {}".format(header[0]))
        payload_length = header[0] - SIZEOF_PACKET_HEADER
        required_len = SIZEOF_PACKET_HEADER + payload_length - len(self.read_buf)
      recved = self.sock.recv(required_len)
      if not recved:
        on_peer_shutdown()
      else:
        self.read_buf += recved
        header = struct.unpack('>HH', self.read_buf[0:SIZEOF_PACKET_HEADER])
        payload_length = header[0] - SIZEOF_PACKET_HEADER
        if SIZEOF_PACKET_HEADER + payload_length == len(self.read_buf):
          # tuple: (packet type, packet payload)
          on_packet_received(header[1] >> 11, header[1] & 0x07FF, self.read_buf[SIZEOF_PACKET_HEADER:])
          self.read_buf = b''

  def send_packet(self, msg_type, packet_data, frame_id=0):
    packet_data_len = len(packet_data)
    pkt = struct.pack('>HH', SIZEOF_PACKET_HEADER + packet_data_len, (msg_type << 11) | (frame_id & 0x7FF)) + packet_data
    self.send_lock.acquire()
    self.sock.sendall(pkt)
    self.send_lock.release()

  @staticmethod
  def parse_fr_frame_packet(payload):
    if len(payload) <= SIZEOF_UINT16:
        raise RuntimeError("parse_fr_frame_packet, payload len error: {}".format(len(payload)))
    frame_payload_len = len(payload) - SIZEOF_UINT16
    t = struct.unpack('>H{}B'.format(frame_payload_len), payload)
    return t[0], t[1:]

  @staticmethod
  def parse_health_packet(payload):
    if len(payload) == 0:
      return None
    if len(payload) % SIZEOF_UINT16 != 0:
      raise RuntimeError("Invalid payload len for health packet: {}".format(len(payload)))
    reg_vals = 6  # unsigned reg val
    signed_reg_vals = 2
    slots_status = 8
    corrections = 4
    if len(payload) // SIZEOF_UINT16 < (reg_vals + corrections):
      raise RuntimeError("Invalid payload len for health packet: {}".format(len(payload)))
    t = struct.unpack('>{}H{}h{}H{}h{}h'.format(
      reg_vals, signed_reg_vals, slots_status, corrections,
      (len(payload) // SIZEOF_UINT16) - reg_vals - signed_reg_vals - slots_status - corrections), payload)
    a_even_cnt = (t[5] & 0x0F00) >> 8
    b_even_cnt = (t[5] & 0xF000) >> 12
    a_odd_cnt = t[5] & 0x000F
    b_odd_cnt = (t[5] & 0x00F0) >> 4
    # PSR0, PSR1, PSR2, PSR3, PIFR0, RateCorrect, OffCorrect
    # ssr0, ssr1, ssr2, ssr3, ssr4, ssr5, ssr6, ssr7
    # max_rate_correction, max_offset_correction, min_rate_correction, min_offset_correction,
    # a_even_cnt, b_even_cnt, a_even_cnt, a_even_cnt, sync frame table
    return t[0], t[1], t[2], t[3], t[4], t[6], t[7], \
           t[8], t[9], t[10], t[11], t[12], t[13], t[14], t[15], \
           t[16], t[17], t[18], t[19], \
           a_even_cnt, b_even_cnt, a_odd_cnt, b_odd_cnt, t[12:]

  @staticmethod
  def parse_error_packet(payload):
    if len(payload) == 0:
      return None
    t = struct.unpack('1H', payload)
    return t[0]

  def send_frame(self, frame_id, payload):
    if len(payload) == 0:
      return 0
    if frame_id not in self.frame_id_to_tx_msg_buf_idx:
      return 0
    msg_buf_idx = self.frame_id_to_tx_msg_buf_idx[frame_id]
    if self.fr_config['TxMsgBufs'][msg_buf_idx]['FrameId'] > self.fr_config['gNumberOfStaticSlots']:
      if len(payload) > self.fr_config['pPayloadLengthDynMax'] * 2:
        # Truncate the byte string
        payload = payload[0:(self.fr_config['pPayloadLengthDynMax'] * 2)]
    else:
      if len(payload) < self.fr_config['gPayloadLengthStatic'] * 2:
        # padding with zero
        payload = payload.ljust(self.fr_config['gPayloadLengthStatic'] * 2, b'\0')
      elif len(payload) > self.fr_config['gPayloadLengthStatic'] * 2:
        # Truncate the byte string
        payload = payload[0:(self.fr_config['gPayloadLengthStatic'] * 2)]
    self.send_packet(PACKET_TYPE_FLEXRAY_FRAME, payload, frame_id=msg_buf_idx)
    return len(payload)

