#!/usr/bin/env python3
import rclpy
import time
from utils.printing import ros_print
from utils.roboclaw_3 import Roboclaw
from utils.servo import MultiServo
from evdev import list_devices, InputDevice, categorize, ecodes

from rclpy.node         import Node
from sensor_msgs.msg    import Image
from geometry_msgs.msg  import Point, Pose, PoseArray, Vector3, Twist
from nav_msgs.msg       import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Bool, Float32

RATE = 10.0  

class MotorNode(Node):
    ADDRESS = 0x80
    FULL_FORWARD = 127
    FULL_BACKWARD = 0
    
    LY = 1
    LX = 0

    def __init__(self, name):
        super().__init__(name)
        
        self.rc = Roboclaw("/dev/ttyACM0", 38400)
        if not self.rc.Open():
            ros_print(self, "Failed to open Roboclaw")
            self.shutdown()
            
        self.ms = MultiServo(init_angles=[0,0,0,0])
        self.speed = 0
        self.turn = 0
        self.pitch = 0
        self.yaw = 0
        self.update = time.time()
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 1)

        # Create a timer to keep calculating/sending commands.
        self.timer = self.create_timer(1 / RATE, self.send_cmd)
        ros_print(self, 'Motor node started')

    def cmd_vel_cb(self, msg: Twist):
        self.update = time.time()
        self.speed = msg.linear.x
        self.turn = msg.angular.z
            
    def send_cmd(self):
        # ros_print(self, time.time() - self.update)
        if time.time() - self.update >= 1:
            self.speed = 0
            self.turn = 0
                        
        left = max(min((-self.speed + self.turn) / 255 * self.FULL_FORWARD + 64, self.FULL_FORWARD), 0)
        right = max(min((-self.speed - self.turn) / 255 * self.FULL_FORWARD + 64, self.FULL_FORWARD), 0)
        
        self.rc.ForwardBackwardM1(self.ADDRESS, int(left))
        self.rc.ForwardBackwardM2(self.ADDRESS, int(right))
        # ros_print(self, 'done')
    
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
    node = MotorNode('actuators')

    # Spin the node until interrupted.
    rclpy.spin(node)

    # Shutdown the node and ROS.
    node.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()