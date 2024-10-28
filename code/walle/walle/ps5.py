#!/usr/bin/env python3
import rclpy
import time
from utils.printing import ros_print
from evdev import list_devices, InputDevice, ecodes
import threading
import numpy as np

from rclpy.node         import Node
from geometry_msgs.msg  import Twist
from std_msgs.msg import Int16MultiArray

RATE = 50.0  

class Ps5Node(Node):
    ADDRESS = 0x80
    FULL_FORWARD = 127
    FULL_BACKWARD = 0
    
    LY = 1
    LX = 0
    RX = 3
    RY = 4
    LT = 2
    RT = 5
    PY = 17
    PX = 16
    LB = 310
    RB = 311
    
    X = 304
    O = 305
    T = 307
    S = 308
    SPECIAL = 315
    
    LEFT_ARM =      0
    RIGHT_ARM =     1
    LOW_NECK =      2
    MID_NECK =      3
    HEAD_SWIVEL =   4
    LEFT_EYE =      5
    RIGHT_EYE =     6
    
    ALARM =         0
    EVA =           1
    TADA =          2
    WALLE =         3
    WOAH =          4

    def __init__(self, name):
        super().__init__(name)
            
        self.dev = InputDevice(list_devices()[0])
        self.cmdvel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.jointpos_pub = self.create_publisher(Int16MultiArray, '/joint_pos', 10)
        self.effects_pub = self.create_publisher(Int16MultiArray, '/effects', 10)
        self.speed = 0
        self.turn = 0
        self.forward = 0
        self.backward = 0
        self.left_arm = 90
        self.right_arm = 90
        self.joints = [0]*7
        self.joints[self.LEFT_ARM] = 90
        self.joints[self.RIGHT_ARM] = 90
        self.joints[self.HEAD_SWIVEL] = 90
        self.joints[self.LOW_NECK] = 50
        self.joints[self.MID_NECK] = 110
        self.joints[self.LEFT_EYE] = 90
        self.joints[self.RIGHT_EYE] = 90
        self.limits = {}
        self.limits[self.LEFT_EYE] = [60, 90]
        self.limits[self.RIGHT_EYE] = [90, 120]
        self.end = False
        self.effects = [0]*5
        self.ps5_thread = threading.Thread(target=self.read_ps5)
        self.ps5_thread.start()
        # Create a timer to keep calculating/sending commands.
        self.timer = self.create_timer(1 / RATE, self.send_vels)
        self.timer = self.create_timer(1 / RATE, self.send_pos)
        ros_print(self, 'PS5 node started')

    def read_ps5(self):
        for event in self.dev.read_loop():
            if event.type == ecodes.EV_ABS:
                self.update = time.time()
                # ros_print(self, event.code, event.value)
                if event.code == self.LX:
                    diff = -(event.value - 127)
                    if abs(diff) < 10: diff = 0
                    self.turn = diff
                elif event.code == self.LT:
                    self.backward = event.value
                elif event.code == self.RT:
                    self.forward = event.value
                elif event.code == self.RX:
                    diff = -(event.value - 127)
                    if abs(diff) < 10: diff = 0
                    self.joints[self.HEAD_SWIVEL] = diff / 128 * 45 + 90
                elif event.code == self.RY:
                    diff = -(event.value - 127)
                    if abs(diff) < 10: diff = 0
                    self.joints[self.MID_NECK] = diff / 128 * 50 + 110
                    diff = -(event.value - 127)
                    if abs(diff) < 10: diff = 0
                    self.joints[self.LOW_NECK] = diff / 128 * 20 + 90
                    # code in the dumbo kinematics
                elif event.code == self.PY:
                    # self.left_arm += 1/1000 * (-event.value)
                    # self.right_arm += 1/1000 * (event.value)
                    # self.left_arm = max(min(self.left_arm, 60), 120)
                    # self.right_arm = max(min(self.right_arm, 60), 120)
                    # self.joints[self.LEFT_ARM] = self.left_arm
                    # self.joints[self.RIGHT_ARM] = self.right_arm
                    self.joints[self.LEFT_ARM] = -event.value * 40 + 90
                    self.joints[self.RIGHT_ARM] = event.value * 40 + 90
                self.speed = (self.forward - self.backward) / 2
                  
            if event.type == ecodes.EV_KEY:
                if event.code == self.LB:
                    self.joints[self.LEFT_EYE] = 90 - event.value * 20
                    if self.LEFT_EYE in self.limits:
                        small, big = self.limits[self.LEFT_EYE]
                        self.joints[self.LEFT_EYE] = min(max(self.joints[self.LEFT_EYE], small), big)
                elif event.code == self.RB:
                    self.joints[self.RIGHT_EYE] = 90 + event.value * 20
                    if self.RIGHT_EYE in self.limits:
                        small, big = self.limits[self.RIGHT_EYE]
                        self.joints[self.RIGHT_EYE] = min(max(self.joints[self.RIGHT_EYE], small), big)
                elif event.code == self.X:
                    self.effects[self.WALLE] = event.value
                elif event.code == self.O:
                    self.effects[self.EVA] = event.value
                elif event.code == self.T:
                    self.effects[self.TADA] = event.value
                elif event.code == self.S:
                    self.effects[self.WOAH] = event.value
            if self.end:
                break
            
            msg = Int16MultiArray()
            msg.data = self.effects
            self.effects_pub.publish(msg)

    def send_pos(self):
        msg = Int16MultiArray()
        msg.data = np.array(self.joints).astype(int).tolist()
        self.jointpos_pub.publish(msg)
        
    def send_vels(self):
        msg = Twist()
        msg.linear.x = float(self.speed)
        msg.angular.z = float(self.turn)
        self.cmdvel_pub.publish(msg)
    
    # Shutdown
    def shutdown(self):
        # No particular cleanup, just shut down the node.
        self.end = True
        self.read_ps5.join()
        self.destroy_node()

#
#   Main Code
#
def main(args=None):

    # Initialize ROS.
    rclpy.init(args=args)

    # Instantiate the detector node.
    node = Ps5Node('play')

    # Spin the node until interrupted.
    rclpy.spin(node)

    # Shutdown the node and ROS.
    node.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()