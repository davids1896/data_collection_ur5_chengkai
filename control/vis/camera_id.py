# -*- coding: utf-8 -*-
"""
Created on Wed Mar 27 10:43:50 2024

@author: Houch
"""

import pyrealsense2 as rs

# 创建一个RealSense设备列表
ctx = rs.context()
devices = ctx.query_devices()

# 遍历设备列表并打印每个设备的ID和类型
for i, dev in enumerate(devices):
    serial_number = dev.get_info(rs.camera_info.serial_number)
    camera_name = dev.get_info(rs.camera_info.name)
    print(f"RealSense相机{i+1}的ID为：{serial_number}")
    print(f"RealSense相机{i+1}的类型为：{camera_name}")
    
    
import pyrealsense2 as rs
import numpy as np

# 初始化 RealSense 摄像头
pipeline = rs.pipeline()
config = rs.config()
pipeline_profile = pipeline.start(config)

# 获取深度图像和彩色图像流的外参矩阵
depth_profile = pipeline_profile.get_stream(rs.stream.depth)
depth_intrinsics = depth_profile.as_video_stream_profile().get_intrinsics()
depth_to_color_extrinsics = depth_profile.get_extrinsics_to(pipeline_profile.get_stream(rs.stream.color))

# 构建外参矩阵
extrinsics_matrix = np.eye(4)
rotation = np.array(depth_to_color_extrinsics.rotation).reshape(3, 3)
translation = np.array(depth_to_color_extrinsics.translation)

extrinsics_matrix[:3, :3] = rotation
extrinsics_matrix[:3, 3] = translation

# 打印外参矩阵
print("点云外参矩阵:")
print(extrinsics_matrix)