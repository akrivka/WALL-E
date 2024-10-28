#!/usr/bin/env python3
import rclpy
import time
from utils.printing import ros_print
from utils.servo import MultiServo
from evdev import list_devices, InputDevice, categorize, ecodes
import threading
import numpy as np

from rclpy.node         import Node
from sensor_msgs.msg    import Image
from geometry_msgs.msg  import Point, Pose, PoseArray, Vector3, Twist
from nav_msgs.msg       import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Int16MultiArray

RATE = 50.0  

class ServoNode(Node):
    LEFT_ARM =      0
    RIGHT_ARM =     1
    LOW_NECK =      2
    MID_NECK =      3
    HEAD_SWIVEL =   4
    LEFT_EYE =      5
    RIGHT_EYE =     6

    def __init__(self, name):
        super().__init__(name)
            
        self.joints = [0]*7
        self.joints[self.LEFT_ARM] = 90
        self.joints[self.RIGHT_ARM] = 90
        self.joints[self.HEAD_SWIVEL] = 90
        self.joints[self.LOW_NECK] = 50
        self.joints[self.MID_NECK] = 110
        self.joints[self.LEFT_EYE] = 90
        self.joints[self.RIGHT_EYE] = 90
        
        self.theta_max = 360 * 2
        self.ms = MultiServo(init_angles=self.joints)
        self.update = time.time()
        self.start = time.time()
        self.T = 0.1
        self.dt = 0.1
        self.last_t = time.time()
        self.create_subscription(Int16MultiArray, '/joint_pos', self.servo_pos_cb, 10)
        self.cycles = 0
        self.last = time.time()
        # # Create a timer to keep calculating/sending commands.
        self.timer = self.create_timer(1 / RATE, self.send_cmd)
        ros_print(self, 'Servo node started')

    def servo_pos_cb(self, msg: Int16MultiArray):
        assert(len(msg.data) == 7)
        self.update = time.time()
        self.filter_update(list(msg.data))
    
    def abs_min(self, u, bound):
        ret = min(abs(u), abs(bound))
        if u < 0:
            return -abs(ret)
        else:
            return abs(ret)
        
    def filter_update(self, new_joints):
        self.dt = min(time.time() - self.last, self.T)
        # ros_print(self, time.time() - self.last)
        self.last = time.time()
        for i in range(len(self.joints)):
            self.joints[i] += self.dt / self.T * (self.abs_min(new_joints[i] - self.joints[i], self.theta_max * self.dt))
            
    def send_cmd(self):
        if time.time() - self.update >= 1:
            return
        # self.cycles += 1
        # self.joints[0] = 90 * np.sin(time.time() - self.start) + 90
        # if time.time() - self.last > 1:
        #     ros_print(self, self.cycles)
        #     self.cycles = 0
        #     self.last = time.time()
                 
        self.ms.angles = self.joints
        ros_print(self, self.joints)
        self.ms.cmd_angles()
        # ros_print(self, self.ms.pwms)
    
    # Shutdown
    def shutdown(self):
        # No particular cleanup, just shut down the node.
        self.destroy_node()

#
#   Main Code
#
def main(args=None):

    # Initialize ROS.
    rclpy.init(args=args)

    # Instantiate the detector node.
    node = ServoNode('servos')

    # Spin the node until interrupted.
    rclpy.spin(node)

    # Shutdown the node and ROS.
    node.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()