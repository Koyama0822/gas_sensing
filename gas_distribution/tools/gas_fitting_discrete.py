#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rosbag
import numpy as np
import sys
import os
import argparse
from nav_msgs.msg import OccupancyGrid
from scipy import interpolate
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import csv

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
parser.add_argument('--model_size', type=int, default=100)
parser.add_argument('--offset_x', type=int, default=25)
parser.add_argument('--offset_y', type=int, default=25) 
args = parser.parse_args()

# load bagfile
bag = rosbag.Bag(args.bagfile)

offset_x = args.offset_x
offset_y = args.offset_y

res    = None
width  = None
height = None
origin = None
map_raw_data = None

for topic, msg, t in bag.read_messages():

    # filter: only need "/estimated_gas_map"
    if topic == '/estimated_gas_map':

        if res == None:
            res    = msg.info.resolution
            width  = msg.info.width
            height = msg.info.height
            origin = msg.info.origin

        raw_map = np.array(list(msg.data)).reshape(width, height)
        print(t)
        



np.set_printoptions(threshold=10000000, linewidth=300)

# careate an extended map
map = np.zeros((args.model_size, args.model_size))
map[args.offset_y:offset_y+height,offset_x:offset_x+width] = raw_map

# modify boundary
map[0]=-1
map[-1]=-1
map[:,0]=-1
map[:,-1]=-1

# create meshgrid
d = np.array(range(args.model_size))
xi, yi = np.meshgrid(d, d) 

x = np.nonzero(map)[0]
y = np.nonzero(map)[1]
z = map[np.nonzero(map)]
z = np.where(z < 0, 0, z)

#   カーネル種類
#   線形 RBF => linear
#   ガウシアン RBF => gaussian
#   多重二乗 RBF => multiquadric
#   逆二乗 RBF => inverse
#   多重調和スプライン RBF => cubic(3次), quintic(5次), thin_plate(薄板スプライン)
#rbf = interpolate.Rbf(x, y, z, function='gaussian') # x, y, z の値で RBF 補間をして曲面を作成する
rbf = interpolate.Rbf(x, y, z, function='multiquadric') # x, y, z の値で RBF 補間をして曲面を作成する
zi = rbf(xi, yi) # x, y, z の曲面における xi, yi の位置の zi を計算する

interp = interpolate.CloughTocher2DInterpolator(list(zip(x, y)), z)
zi = interp(xi, yi)

fig = plt.figure(figsize=(18, 7), dpi=200) # 画像を作成する
 
# 曲面のプロット ==================================================================
ax = fig.add_subplot(121, projection='3d') # 1 × 2 の 1 枚目に描画する
ax.plot_surface(xi, yi, zi)                # サーフェスの描画
ax.scatter(x,y,z,color='red',s=1)
el = 100                                   # 視点高さを設定する
ax.view_init(elev=el, azim=90)             # ビューの設定
ax.set_title('elev = 100, deg = 90')       # タイトルの設定
ax.set_xlabel('xi')                        # 軸ラベルの設定
ax.set_ylabel('yi')                        # 軸ラベルの設定
ax.set_zlabel('zi')                        # 軸ラベルの設定
 
# コンター =======================================================================
ax = fig.add_subplot(122) # 1 × 2 の 2 枚目に描画する
contour = ax.contourf(xi, yi, zi)
fig.colorbar(contour)
ax.set_xlim([d.max(), d.min()])
ax.set_ylim([d.max(), d.min()])
ax.set_title('contour')       # タイトルの設定
ax.set_xlabel('xi')            # 軸ラベルの設定
ax.set_ylabel('yi')            # 軸ラベルの設定
 
# グラフ出力c
file_name = '3D and Contour.jpg' # グラフ名設定
plt.savefig(file_name)           # グラフ出力
 
plt.show()
