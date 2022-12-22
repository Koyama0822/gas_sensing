#!/usr/bin/env python
import rosbag
import numpy as np
import sys
import os
import argparse
from nav_msgs.msg import OccupancyGrid

"""
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
nav_msgs/MapMetaData info
  time map_load_time
  float32 resolution
  uint32 width
  uint32 height
  geometry_msgs/Pose origin
    geometry_msgs/Point position
      float64 x
      float64 y
      float64 z
    geometry_msgs/Quaternion orientation
      float64 x
      float64 y
      float64 z
      float64 w
int8[] data
"""

# python parsing
parser = argparse.ArgumentParser() 
parser.add_argument('--bagfile', type=str)
parser.add_argument('--time_offset', type=int, default=0) 
args = parser.parse_args()

# load bagfile
bag = rosbag.Bag(args.bagfile)

res    = None
width  = None
height = None
origin = None
map_raw_data = None
start_time = None

for topic, msg, t in bag.read_messages():

    if start_time == None:
        start_time = t

    if (t - start_time).to_sec() < args.time_offset:
        continue

    # filter: only need "/estimated_gas_map"
    if topic == '/estimated_gas_map':

        if res == None:
            res    = msg.info.resolution
            width  = msg.info.width
            height = msg.info.height
            origin = msg.info.origin

        
        map_raw_data = msg.data

map = np.array(map_raw_data).reshape(width, height)
np.set_printoptions(threshold=10000000, linewidth=200)
print("map: {}".format(map))

### TODO1: get high resolution map

### TODO2: gizagiza route

### TODO3: (main)
### koyama algorithm: estimate the peak value from this map, that should be the gas source

        



