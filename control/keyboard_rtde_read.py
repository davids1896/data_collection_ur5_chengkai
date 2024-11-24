from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface
from pynput import keyboard
import time
from math import pi
import numpy as np
import copy
from scipy.spatial.transform import rotation, Rotation
from utils import vec2euler, euler2quat, vec2mat
# from robotiq_gripper import RobotiqGripper
from vis import  RealSense_Camera
import pickle
import os
import select
from termcolor import cprint
import sys
import struct
import serial

def precise_sleep(dur, tol=1e-3):
    st_time = time.monotonic()
    time.sleep(dur - tol)
    while time.monotonic() - st_time < dur:
        pass
    

global gripper_opened, ser
gripper_opened = False

def openSerial(port, baudrate):
    global ser
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = baudrate
    ser.open()
    return ser

def data2bytes(data):
    rdata = [0xff]*2
    if data == -1:
        rdata[0] = 0xff
        rdata[1] = 0xff
    else:
        rdata[0] = data & 0xff
        rdata[1] = (data >> 8) & 0xff
    return rdata

def num2str(num):
    str = hex(num)[2:4]
    if len(str) == 1:
        str = '0' + str
    return bytes.fromhex(str)

def checknum(data, leng):
    result = 0
    for i in range(2, leng):
        result += data[i]
    return result & 0xff

def toggle_gripper():
    global gripper_opened
    datanum = 0x03
    b = [0] * (datanum + 5)
    b[0] = 0xEB
    b[1] = 0x90
    b[2] = 1
    b[3] = datanum
    b[4] = 0x11 if gripper_opened else 0x10
    b[5] = data2bytes(500)[0]
    b[6] = data2bytes(500)[1]
    b[7] = checknum(b, datanum + 4)
    putdata = b''
    for i in range(1, datanum + 6):
        putdata += num2str(b[i - 1])
    ser.write(putdata)
    getdata = ser.read(7)
    gripper_opened = not gripper_opened




class Service(object):
    def __init__(self, ip, acc=0.5, vel=0.5, frequency=125):
        #self.realsense_camera = RealSense_Camera(type="D435I", id ="213622078748" )
        #self.realsense_camera.prepare()
        self.rtde_c = RTDEControlInterface(ip)
        self.rtde_r = RTDEReceiveInterface(ip)
        self.ser = openSerial('COM9', 115200)
        self.acc = acc
        self.vel = vel   
        self.dt = 1. / frequency
        self.lookahead_time = 0.1
        self.gain = 500
        self.gripper_opened = True
        # defined in joint space
        self.init_pose = [-0.5*pi, -pi/2, -pi/2, -pi/2, pi/2, 0]
        variable_list = [[0, 0, 0, 0, 0, 0]]
        point_cloud_list = []
        tcp_pose  = []
        joint_positions = []
        gripper_state  = []
        image_list = []
        depth_list = []
        self.listener = keyboard.Listener(on_press=lambda key: Service._on_press(self, key,variable_list,point_cloud_list,
                                                                                 tcp_pose,image_list,depth_list,
                                                                                 joint_positions,gripper_state))
        #self.write_to_file = 'output.txt'
        self.realsense_camera = RealSense_Camera(type="L515", id ="f1470417" )
        #self.realsense_camera = RealSense_Camera(type="D435I", id ="213622078748" )
        self.realsense_camera.prepare()
    
    def loop(self):
        print("Starting loop...")
        self.listener.start()
        self.listener.join()
        
    def save_state(self, point_cloud_list, image_list, depth_list, action_list,  tcp_pose,joint_positions,gripper_state,save_dir = 'output/luo'):
        os.makedirs(save_dir, exist_ok=True)
        
        point_cloud_arrays = np.stack(point_cloud_list, axis=0)
        image_arrays = np.stack(image_list, axis=0)
        depth_arrays = np.stack(depth_list, axis=0)
        #robot_state_arrays = np.stack(robot_state_list, axis=0)
        action_arrays = np.stack(action_list, axis=0)
      
        tcp_pose = np.stack(tcp_pose, axis=0)
        joint_positions = np.stack(joint_positions, axis=0)
        gripper_state = np.stack(gripper_state, axis=0)
        
        data = {
            'point_cloud': point_cloud_arrays,
            'image': image_arrays,
            'depth': depth_arrays,
            'action': action_arrays,
            'tool pose': tcp_pose,
            'joint pose': joint_positions,
            'gripper pose': gripper_state
        }
        
        with open(os.path.join(save_dir, 'data_10.pkl'), 'wb') as f:
            pickle.dump(data, f)
        
        print("save data to: ", save_dir)
        return save_dir       

    def close(self):
        self.listener.stop()
        self.rtde_c.stopScript()

    def _translate(self, vec):
        tcp = self.rtde_r.getActualTCPPose()
        t_start = self.rtde_c.initPeriod()
        tcp [:3] = [a + b for (a, b) in zip(tcp, vec)]
        #target_pose = [a + b for (a, b) in zip(tcp, vec)]
        self.rtde_c.servoL(tcp, self.vel, self.acc, self.dt, self.lookahead_time, self.gain)
        #self.rtde_c.moveL(tcp, 2.0,self.acc)
        self.rtde_c.waitPeriod(t_start)
    
    def _on_press(self,key,variable_list,point_cloud_list,tcp_pose,image_list,depth_list,joint_positions,gripper_state):
            
        arm_action = variable_list[-1]
        try:
            key = key.char
        except:
            key = key
        finally:
            if key == 'w':
                print('\nW pressed (Up)')
                self._translate((0, 0, 0.04))
                delta_arm_action = [0, 0, 0.04, 0, 0, 0]
                arm_action = [i+j for i,j in zip(arm_action, delta_arm_action)]
            elif key == 's':
                print('\nS pressed (Down)')
                self._translate((0, 0, -0.04))
                delta_arm_action = [0, 0, -0.04, 0, 0, 0]
                arm_action = [i+j for i,j in zip(arm_action, delta_arm_action)]
            elif key == 'a':
                print('\nA pressed (Left)')
                self._translate((0, 0.04, 0))
                delta_arm_action = [0, 0.04,0, 0, 0, 0]
                arm_action = [i+j for i,j in zip(arm_action, delta_arm_action)]
            elif key == 'd':
                print('\nD pressed (Right)')
                self._translate((0, -0.04, 0))
                delta_arm_action = [0, -0.04,0, 0, 0, 0]
                arm_action = [i+j for i,j in zip(arm_action, delta_arm_action)]
            elif key == 'q':
                print('\nQ pressed (Front)')
                self._translate((0.06, 0, 0))
                delta_arm_action = [0.06, 0, 0, 0, 0, 0]
                arm_action = [i+j for i,j in zip(arm_action, delta_arm_action)]
            elif key == 'e':
                print('\nE pressed (Back)')
                self._translate((-0.06, 0, 0))
                delta_arm_action = [-0.06, 0, 0, 0, 0, 0]
                arm_action = [i+j for i,j in zip(arm_action, delta_arm_action)]
            elif key == 'r':
                print('\nR pressed (Reset)')
                self.rtde_c.servoStop()
                self.rtde_c.moveJ(self.init_pose)
                delta_arm_action = [0, 0, 0, 0, 0, 0]
                arm_action = [i+j for i,j in zip(arm_action, delta_arm_action)]
            elif key == 'g':
                print('\nG pressed (Grasp)')
                self.gripper_opened = toggle_gripper(self.ser, self.gripper_opened)
            elif key == keyboard.Key.esc:
                 print('Exiting the program...')
                 return False
            
            if key in ['w','s','a','d','q','e','g']:
               variable_list.append(arm_action)
               #print('Final variable list:', variable_list)
            
               point_cloud, rgbd_frame = self.realsense_camera.get_frame()
               rgb = rgbd_frame[:, :, :3]
               depth = rgbd_frame[:, :, -1]
               obs_dict = {     
                   'point_cloud': point_cloud,
                   'image': rgb,
                   'depth': depth}
               point_cloud = obs_dict['point_cloud']
               image = obs_dict['image']
               depth = obs_dict['depth']  
               import matplotlib.pyplot as plt
               from mpl_toolkits.mplot3d import Axes3D
               fig = plt.figure(figsize=(12, 6))

               ax1 = fig.add_subplot(121)
               ax1.imshow(image)
               ax1.axis('off')
               color = point_cloud[:,3:6].max()
               ax2 = fig.add_subplot(122, projection='3d')
               ax2.scatter(point_cloud[:, 1], point_cloud[:, 0], point_cloud[:, 2], c=point_cloud[:,3:6]/color, s=2)
               ax2.set_xlabel('X')
               ax2.set_ylabel('Y')
               ax2.set_zlabel('Z')
               step = len(variable_list)-1
               plt.suptitle(f"step: {step}")
               plt.tight_layout()
               plt.show()
               point_cloud_list.append(point_cloud)
               #import visualizer
               #visualizer.visualize_pointcloud(point_cloud)
               #robot_state_list.append(g_r)
               image_list.append(image)
               depth_list.append(depth)
               print(len(point_cloud_list))
               p_l = self.rtde_r.getActualTCPPose()
               p_j = self.rtde_r.getActualQ()
            #    g_r = self._gripper.get_current_position()
               print("============================")
               print("Current tool pose:", p_l)
               print("Current joint pose:", p_j)
               print("Current gripper pose:", g_r)
               print("============================")
               tcp_pose.append(p_l)
               joint_positions.append(p_j)
               gripper_state.append(gripper_opened)
            if key == "c":
                self.save_state(point_cloud_list, image_list, depth_list,  variable_list, tcp_pose,joint_positions,gripper_state,save_dir = r"D:\output\chicken")
                
               #import visualizer
               #visualizer.visualize_pointcloud(point_cloud)
               
    
if __name__ == "__main__":

    service = Service(ip='192.169.0.10')
    try:
        service.loop()
    finally:
        service.close()
    #service.collect_demo()
