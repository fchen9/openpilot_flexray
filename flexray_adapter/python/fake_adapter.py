import struct
import socket
from select import select
from constants import *

read_buf = b''


def receive_packet(conn, on_packet_received):
  global read_buf
  readables, _, _ = select([conn], [], [], 1.)
  if len(readables) <= 0:
    return True
  if len(read_buf) < SIZEOF_PACKET_HEADER:
    required_len = SIZEOF_PACKET_HEADER - len(read_buf)
  else:
    header = struct.unpack('>HH', read_buf[0:SIZEOF_PACKET_HEADER])
    if header[0] < SIZEOF_PACKET_HEADER:
      raise RuntimeError("Invalid packet length: {}".format(header[0]))
    payload_length = header[0] - SIZEOF_PACKET_HEADER
    required_len = SIZEOF_PACKET_HEADER + payload_length - len(read_buf)
  try:
    recved = conn.recv(required_len)
    if not recved:
      return False
    else:
      read_buf += recved
      header = struct.unpack('>HH', read_buf[0:SIZEOF_PACKET_HEADER])
      payload_length = header[0] - SIZEOF_PACKET_HEADER
      if SIZEOF_PACKET_HEADER + payload_length == len(read_buf):
        # tuple: (packet type, packet payload)
        on_packet_received(header[1] >> 11, header[1] & 0x07FF, read_buf[SIZEOF_PACKET_HEADER:])
        read_buf = b''
  except ConnectionResetError:
    print('Peer reset the connection.')
    return False
  except ConnectionAbortedError:
    print('Connection aborted.')
    return False
  return True

def on_packet_received(pkt_type, frame_id, payload):
  print('Received packet type {}, frame id {}, {} bytes'.format(pkt_type, frame_id, len(payload)))

def send_packet(conn, msg_type, packet_data, frame_id=0):
  packet_data_len = len(packet_data)
  pkt = struct.pack('>HH', SIZEOF_PACKET_HEADER + packet_data_len, (msg_type << 11) | (frame_id & 0x7FF)) + packet_data
  conn.sendall(pkt)

import random
if __name__ == '__main__':
  s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  s.bind(('localhost', 3888))
  s.listen(1)
  while True:
    print('Listening...')
    conn, addr = s.accept()
    conn.setblocking(0)
    sent_result = True
    while True:
      if not receive_packet(conn, on_packet_received):
        break
      if not sent_result and random.randint(0, 100) % 99 == 0:
        send_packet(conn, PACKET_TYPE_FLEXRAY_JOIN_CLUSTER_FAILED, b'')
        print('Send fake join cluster failed packet.')
        sent_result = True
      if not sent_result and random.randint(0, 21) % 20 == 0:
        send_packet(conn, PACKET_TYPE_FLEXRAY_JOINED_CLUSTER, b'')
        print('Send fake join cluster succeeded packet.')
        sent_result = True
      #conn.sendall(data)
    conn.close()