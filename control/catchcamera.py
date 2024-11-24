# -*- coding: utf-8 -*-
"""
Created on Wed Apr 24 10:42:43 2024

@author: Houch
"""

from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface
from robotiq_gripper import RobotiqGripper
from vis import  RealSense_Camera
import numpy as np  
        # configure env
ip='192.169.0.10' 
rtde_c = RTDEControlInterface(ip)
rtde_r = RTDEReceiveInterface(ip)
_gripper = RobotiqGripper()
_gripper.connect(ip, 63352)
_gripper.activate()
_gripper_close = False
realsense_camera = RealSense_Camera(type="L515", id ="f1470417" )
realsense_camera.prepare()
point_cloud, rgbd_frame = realsense_camera.get_frame()
rgb = rgbd_frame[:, :, :3]
p_l = rtde_r.getActualTCPPose()
p_j = rtde_r.getActualQ()
g_r = _gripper.get_current_position()
print("============================")
print("Current tool pose:", p_l)
print("Current joint pose:", p_j)
print("Current gripper pose:", g_r)
print("============================")
p_l=np.concatenate((p_j,p_l))
p_l=np.concatenate((p_l,np.array([g_r])))

obs_dict = {}
obs_dict['point_cloud'] = point_cloud.tolist()
obs_dict['agent_pos'] = p_l.tolist()
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
fig = plt.figure(figsize=(12, 6))

ax1 = fig.add_subplot(121)
ax1.imshow(rgb)
ax1.axis('off')
color = point_cloud[:,3:6].max()
ax2 = fig.add_subplot(122, projection='3d')
ax2.scatter(point_cloud[:, 1], point_cloud[:, 0], point_cloud[:, 2], c=point_cloud[:,3:6]/color, s=2)
ax2.set_xlabel('X')
ax2.set_ylabel('Y')
ax2.set_zlabel('Z')
plt.show()
import json

# 将字典保存为 JSON 文件
with open('data3.json', 'w') as json_file:
    json.dump(obs_dict, json_file)

print("字典已保存为 data3.json 文件")
