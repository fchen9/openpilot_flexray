#!/usr/bin/env python
import os
from common.basedir import BASEDIR
os.environ['BASEDIR'] = BASEDIR
import zmq
import numpy as np
import sys
from selfdrive.messaging import sub_sock, recv_one
from selfdrive.services import service_list

import inspect


def do_test():
  context = zmq.Context()
  gps_external_sock = sub_sock(context, service_list['gpsLocationExternal'].port, conflate=True)
  gps_external_sock_new = sub_sock(context, 9032, conflate=True)
  ublox_gnss_sock = sub_sock(context, service_list['ubloxGnss'].port, conflate=True)
  ublox_gnss_sock_new = sub_sock(context, 9033, conflate=True)



  while True:
    old_gps = recv_one(gps_external_sock).gpsLocationExternal
    gps = recv_one(gps_external_sock_new).gpsLocationExternal
    attrs = ['flags', 'latitude', 'longitude', 'altitude', 'speed', 'bearing',
      'accuracy', 'timestamp', 'source', 'vNED', 'verticalAccuracy', 'bearingAccuracy', 'speedAccuracy']
    for attr in attrs:
      o = getattr(old_gps, attr)
      n = getattr(gps, attr)
      if attr == 'vNED':
        if len(o) != len(n):
          print('Gps vNED len mismatch', o, n)
        else:
          for i in range(len(o)):
            if abs(o[i] - n[i]) > 1e-3:
              print('Gps vNED mismatch', o, n)
      elif o != n:
        print('Gps mismatch', attr, o, n)

    old_gnss = recv_one(ublox_gnss_sock).ubloxGnss
    gnss = recv_one(ublox_gnss_sock_new).ubloxGnss
    if old_gnss.which() == 'measurementReport' and gnss.which() == 'measurementReport':
      if str(old_gnss.measurementReport) == str(gnss.measurementReport):
        print('Gnss measurementReport matched!')
      else:
        print('Gnss measurementReport mismatched!')
        print('gnss measurementReport old', old_gnss.measurementReport.measurements)
        print('gnss measurementReport new', gnss.measurementReport.measurements)
    elif old_gnss.which() == 'ephemeris' and gnss.which() == 'ephemeris':
      if str(old_gnss.ephemeris) == str(gnss.ephemeris):
        print('Gnss ephemeris matched!')
      else:
        print('Gnss ephemeris mismatched!')
        print('gnss ephemeris old', old_gnss.ephemeris)
        print('gnss ephemeris new', gnss.ephemeris)


if __name__ == "__main__":
  do_test()
