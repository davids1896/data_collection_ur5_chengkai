# -*- coding: utf-8 -*-
"""
Created on Tue May 28 14:11:32 2024

@author: Houch
"""

from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface
from pynput import keyboard
import time
from math import pi
import numpy as np
import copy
from scipy.spatial.transform import rotation, Rotation
from utils import vec2euler, euler2quat, vec2mat
from robotiq_gripper import RobotiqGripper
from vis import  RealSense_Camera
import pickle
import os
import select
from termcolor import cprint
import sys


def precise_sleep(dur, tol=1e-3):
    st_time = time.monotonic()
    time.sleep(dur - tol)
    while time.monotonic() - st_time < dur:
        pass
    




class Service(object):
    def __init__(self, ip, acc=0.5, vel=0.5, frequency=125):
        #self.realsense_camera = RealSense_Camera(type="D435I", id ="213622078748" )
        #self.realsense_camera.prepare()
        self.rtde_c = RTDEControlInterface(ip)
        self.rtde_r = RTDEReceiveInterface(ip)
        self._gripper = RobotiqGripper()
        self._gripper.connect(ip, 63352)
        self.acc = acc
        self.vel = vel   
        self.dt = 1. / frequency
        self.lookahead_time = 0.1
        self.gain = 500
        self._gripper.activate()
        self._gripper_close = False
        # defined in joint space
        self.init_pose = [0.75*pi, -pi/2, pi/2, -pi/2, -pi/2, 0]
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
        #self.realsense_camera = RealSense_Camera(type="L515", id ="f1470417" )
        #self.realsense_camera = RealSense_Camera(type="D435I", id ="213622078748" )       
        #self.realsense_camera.prepare()
    
    def loop(self):
        print("Starting loop...")
        self.listener.start()
        self.listener.join()
        
     

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
                self._translate((0, 0, 0.02))
                
            elif key == 's':
                print('\nS pressed (Down)')
                self._translate((0, 0, -0.02))
               
            elif key == 'a':
                print('\nA pressed (Left)')
                self._translate((0, 0.04, 0))
               
            elif key == 'd':
                print('\nD pressed (Right)')
                self._translate((0, -0.02, 0))
                
            elif key == 'q':
                print('\nQ pressed (Front)')
                self._translate((0.02, 0, 0))
                
            elif key == 'e':
                print('\nE pressed (Back)')
                self._translate((-0.02, 0, 0))
                
            elif key == 'r':
                print('\nR pressed (Reset)')
                self.rtde_c.servoStop()
                self.rtde_c.moveJ(self.init_pose)
               
            elif key == 'g':
                print('\nG pressed (Grasp)')
                if self._gripper_close:
                    self._gripper.move(0, 255, 0)
                    delta_arm_action = [0, 0, 0, 0, 255, 0]
                    arm_action = [i+j for i,j in zip(arm_action, delta_arm_action)]
                else:
                    self._gripper.move(255, 255, 0)
                    delta_arm_action = [0, 0, 0, 255, 255, 0]
                    #variable_list.append(delta_arm_action)
                    arm_action = [i+j for i,j in zip(arm_action, delta_arm_action)]
                self._gripper_close = not self._gripper_close
            elif key == keyboard.Key.esc:
                   print('Exiting the program...')
                   return False
            
               #visualizer.visualize_pointcloud(point_cloud)
               
    
if __name__ == "__main__":

    service = Service(ip='192.169.0.10')
    try:
        service.loop()
    finally:
        service.close()
    #service.collect_demo()
