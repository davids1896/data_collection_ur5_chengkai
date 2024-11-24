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

def precise_sleep(dur, tol=1e-3):
    st_time = time.monotonic()
    time.sleep(dur - tol)
    while time.monotonic() - st_time < dur:
        pass

class Service(object):
    def __init__(self, ip, acc=0.5, vel=0.5, frequency=125):
        self.rtde_c = RTDEControlInterface(ip)
        self.rtde_r = RTDEReceiveInterface(ip)
        self.acc = acc
        self.vel = vel
        self.dt = 1. / frequency
        self.lookahead_time = 0.1
        self.gain = 500
        #self._gripper = RobotiqGripper()
        #self._gripper.connect(ip, 63352)
        #self._gripper.activate()
        #self._gripper_close = False
        ## defined in joint space
        self.init_pose = [-0.5*pi, -pi/2, -pi/2, -pi/2, pi/2, 0]
        self.listener = keyboard.Listener(on_press=lambda key: Service._on_press(self, key))

    def loop(self):
        print("Starting loop...")
        st_time = time.time()
        self.listener.start()
        self.listener.join()
                
                
               

    def close(self):
        self.listener.stop()
        self.rtde_c.stopScript()

    def _translate(self, vec):
        tcp = self.rtde_r.getActualTCPPose()
        t_start = self.rtde_c.initPeriod()
        tcp [:3] = [a + b for (a, b) in zip(tcp, vec)]
        self.rtde_c.servoL(tcp, self.vel, self.acc, self.dt, self.lookahead_time, self.gain)
        self.rtde_c.waitPeriod(t_start)

    def _on_press(self, key):
        try:
            key = key.char
        except:
            key = key
        finally:
            if key == 'w':
                print('\nW pressed (Up)')
                self._translate((0, 0, 0.01))
            elif key == 's':
                print('\nS pressed (Down)')
                self._translate((0, 0, -0.01))
            elif key == 'a':
                print('\nA pressed (Left)')
                self._translate((0, 0.01, 0))
            elif key == 'd':
                print('\nD pressed (Right)')
                self._translate((0, -0.01, 0))
            elif key == 'q':
                print('\nQ pressed (Front)')
                self._translate((0.01, 0, 0))
            elif key == 'e':
                print('\nE pressed (Back)')
                self._translate((-0.01, 0, 0))
            elif key == 'r':
                print('\nR pressed (Reset)')
                self.rtde_c.servoStop()
                self.rtde_c.moveJ(self.init_pose)
           # elif key == 'g':
            #    print('\nG pressed (Grasp)')
            #    if self._gripper_close:
            #        self._gripper.move(0, 255, 0)
            #    else:
            #        self._gripper.move(255, 255, 0)
            #    self._gripper_close = not self._gripper_close


if __name__ == "__main__":

    service = Service(ip='192.169.0.10')
    try:
        service.loop()
    finally:
        service.close()
