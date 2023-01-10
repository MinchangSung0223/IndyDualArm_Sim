import pybullet as p
import time
import math
from datetime import datetime
from numpy import *
from pylab import *
import struct
import sys
import os, fnmatch
import argparse
from time import sleep


def readLogFile(filename, verbose=True):
  f = open(filename, 'rb')

  print('Opened'),
  print(filename)

  keys = f.readline().decode('utf8').rstrip('\n').split(',')
  fmt = f.readline().decode('utf8').rstrip('\n')

  # The byte number of one record
  sz = struct.calcsize(fmt)
  # The type number of one record
  ncols = len(fmt)

  if verbose:
    print('Keys:'),
    print(keys)
    print('Format:'),
    print(fmt)
    print('Size:'),
    print(sz)
    print('Columns:'),
    print(ncols)

  # Read data
  wholeFile = f.read()
  # split by alignment word
  chunks = wholeFile.split(b'\xaa\xbb')
  log = list()
  for chunk in chunks:
    if len(chunk) == sz:
      values = struct.unpack(fmt, chunk)
      record = list()
      for i in range(ncols):
        record.append(values[i])
      log.append(record)

  return log

import pybullet_data

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
robotId = p.loadURDF("model/IndyDualArm.urdf", [0, 0, 0.0],[0, 0, 0, 1],flags=p.URDF_USE_SELF_COLLISION)
planeId = p.loadURDF("plane.urdf",[0, 0, -0.521])

log = readLogFile("data.bin")
right_num_list = [2,3,4,5,6,7]
left_num_list = [10,11,12,13,14,15]
for record in log:
  qNum = record[16]
  q_r = record[17:17+6]
  q_l = record[23:23+6]
  for i in range(0,6):
    p.resetJointState(robotId,right_num_list[i],q_r[i])
    p.resetJointState(robotId,left_num_list[i],q_l[i])
  sleep(0.0001)
