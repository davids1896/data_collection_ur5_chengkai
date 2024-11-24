# -*- coding: utf-8 -*-
"""
Created on Mon Apr  1 15:48:07 2024

@author: Houch
"""
from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface
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


class Service(object):
    def __init__(self, ip, acc=0.5, vel=0.5, frequency=125):
        self.rtde_c = RTDEControlInterface(ip)
        self.rtde_r = RTDEReceiveInterface(ip)
        self.acc = acc
        self.vel = vel
        self.dt = 1. / frequency
        self.lookahead_time = 0.1
        self.gain = 500
        self._gripper = RobotiqGripper()
        self._gripper.connect(ip, 63352)
        self._gripper.activate()
        self._gripper_close = False
        # defined in joint space
        self.init_pose = [0.75*pi, -pi/2, pi/2, -pi/2, -pi/2, 0]
        
    def _translate(self, vec):
          tcp = self.rtde_r.getActualTCPPose()
          t_start = self.rtde_c.initPeriod()
          tcp [:3] = [a + b for (a, b) in zip(tcp, vec)]
          self.rtde_c.servoL(tcp, self.vel, self.acc, self.dt, self.lookahead_time, self.gain)
          self.rtde_c.waitPeriod(t_start)
      
    def robot_action(self,action, st_time):
        if st_time == "True":
           self.rtde_c.servoStop()
           self.rtde_c.moveJ(self.init_pose)

           step = abs(int(action[0]/0.02))
       
           for i in range(step):
               if action[0]<0:
                  self._translate((-0.02, 0, 0))
               
               else: 
                   self._translate((0.02, 0, 0))
               time.sleep(1)
           step = abs(int(action[1]/0.02))
        
           for i in range(step):
               if action[1]<0:
                  self._translate((0, -0.02,0))
               else: 
                  self._translate((0, 0.02,0))
               time.sleep(1)
           step = abs(int(action[2]/0.02))
        
           for i in range(step):
               if action[2]<0:
                 self._translate((0,0,-0.02))
               else: 
                  self._translate((0,0,0.02))
               time.sleep(1)
           self._gripper.move(255, 255, 0) 
        else: 
            step = abs(int(action[2]/0.02))
            for i in range(step):
                 if action[2]<0:
                   self._translate((0,0,-0.02))
                 else: 
                    self._translate((0,0,0.02))
                 time.sleep(1)
            step = abs(int(action[1]/0.02))
            for i in range(step):
                if action[1]<0:
                   self._translate((0, -0.02,0))
                else: 
                   self._translate((0, 0.02,0))
                time.sleep(1)
            step = abs(int(action[0]/0.02))
            for i in range(step):
                if action[0]<0:
                   self._translate((-0.02, 0, 0))
                
                else: 
                    self._translate((0.02, 0, 0))
                time.sleep(1)
            self._gripper.move(0, 255, 0)
        #self.rtde_c.servoStop()
       # self.rtde_c.moveJ(self.init_pose)

if __name__ == "__main__":
     action = [-1.3858e-01, -0.08, -0.1480255,  2.1496e+02,  1.6391e+02,
          -2.7738e-02]
     #action = [-9.4451e-02, -1.4375e-01, -1.1038e-01,  1.2946e+02,  1.1474e+02,
     #      4.3283e-03]
     #action = [6.0000e-02, -1.3368e-01, -1.1038e-01,  2.4314e+02,  3.7844e+02,
     #    -4.3689e-02]
     #action = [-0.12, -0.16, -0.14,  2.1496e+02,  1.6391e+02,
     #     -2.7738e-02]
     service = Service(ip='192.169.0.10') 
     st_time = time.time()
     service.robot_action(action, "True")
     action = [1.3858e-01, 0.16, 0.08,  2.1496e+02,  1.6391e+02,
          -2.7738e-02]
     #action = [9.4451e-02, 1.4375e-01, 1.1038e-01,  1.2946e+02,  1.1474e+02,
     #      4.3283e-03]
     #action = [-6.0000e-02, 0.1, 0.08,  2.4314e+02,  3.7844e+02,
     #    -4.3689e-02]
     #action = [0.12, 0.08, 0.08,  2.1496e+02,  1.6391e+02,
     #    -2.7738e-02]
     service.robot_action(action, "False")