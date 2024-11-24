# import open3d as o3d
#o3d.t.io.RealSenseSensor.list_devices()
#列出相机信息
import numpy as np
import math
import time
import cv2
import pyrealsense2 as rs
# import visualizer
# vis = visualizer.Visualizer()
from vis.camera_utils import CameraInfo, create_point_cloud_from_depth_image, transform_point_cloud

class RealSense_Camera:
    def __init__(self, type="L515", id=None):
        self.pc = rs.pointcloud()
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        if id is not None:
            self.config.enable_device(id)
        # if type == "L515":
        #     self.config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
        # else:
        self.config.enable_stream(rs.stream.color, 640, 480,rs.format.bgr8, 30)
        self.profile = self.pipeline.start(self.config)
        # Getting the depth sensor's depth scale (see rs-align example for explanation)
        self.depth_sensor = self.profile.get_device().first_depth_sensor()
        self.depth_scale = self.depth_sensor.get_depth_scale()
        print("Depth Scale is: " , self.depth_scale)

        # We will be removing the background of objects more than
        #  clipping_distance_in_meters meters away
        self.clipping_distance_in_meters = 1 #1 meter
        self.clipping_distance = self.clipping_distance_in_meters / self.depth_scale

        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)

    def prepare(self):
        for fid in range(200):
            frames = self.pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()

    def get_frame(self, remove_bg = False):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        profile = aligned_frames.get_profile()
        intrinsics = profile.as_video_stream_profile().get_intrinsics()
        camera = CameraInfo(intrinsics.width, intrinsics.height, intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy)
        depth_frame = aligned_frames.get_depth_frame() 
        color_frame = aligned_frames.get_color_frame()
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        color_image[:, :, [0,2]] = color_image[:,:,[2,0]]
        transform = np.array([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
        
        if not remove_bg:
            
            point_xyz = create_point_cloud_from_depth_image(depth_image, camera, organized=False)
            point_xyz = transform_point_cloud(point_xyz, transform)
            rgbd_frame = np.concatenate([color_image, np.expand_dims(depth_image, axis=-1)], axis = -1)
            point_color = color_image.reshape(-1, 3)


        else:
            # Remove background - Set pixels further than clipping_distance to grey
            grey_color = 153
            depth_image_3d = np.dstack((depth_image,depth_image,depth_image)) #depth image is 1 channel, color is 3 channels
            bg_removed = np.where((depth_image_3d > self.clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)
            point_xyz = create_point_cloud_from_depth_image(depth_image, camera, organized=False)
            transform = np.array([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
            point_xyz = transform_point_cloud(point_xyz, transform)
            point_color = bg_removed.reshape([-1,3])
            point_color = transform_point_cloud(point_color, transform)
            rgbd_frame = np.concatenate([bg_removed, np.expand_dims(depth_image, axis=-1)], axis = -1)

        point_cloud = np.concatenate([point_xyz, point_color], axis = 1)

        return point_cloud, rgbd_frame


        

  