#!/usr/bin/env python3
import rclpy
import time
from utils.printing import ros_print
from evdev import list_devices, InputDevice, ecodes
import numpy as np
from playsound import playsound


from rclpy.node         import Node
from geometry_msgs.msg  import Twist
from std_msgs.msg import Int16MultiArray

RATE = 50.0  

class EffectsNode(Node):
    
    ALARM = 0
    EVA = 1
    TADA = 2
    WALLE = 3
    WOAH = 4
    def __init__(self, name):
        super().__init__(name)
            
        self.dev = InputDevice(list_devices()[0])
        self.sounds = {self.ALARM : False,
                       self.EVA : False,
                       self.TADA : False,
                       self.WALLE : False,
                       self.WOAH : False}
        self.paths = {self.ALARM : 'alarm',
                       self.EVA : 'eva',
                       self.TADA : 'tada',
                       self.WALLE : 'walle',
                       self.WOAH : 'woah'}
        self.create_subscription(Int16MultiArray, '/effects', self.cb_effects, 10)
        ros_print(self, 'Effects node started')
        
    def cb_effects(self, msg):
        for incoming, effect in zip(msg.data, self.sounds):
            ros_print(self, effect, incoming, bool(incoming), self.sounds[effect])
            if bool(incoming) != self.sounds[effect]:
                self.sounds[effect] = bool(incoming)
                if bool(self.sounds[effect]):
                    ros_print(self, 'playing an effects')
                    playsound(f'/home/walle/robot_ws/src/walle/walle/resources/sounds/{self.paths[effect]}.mp3')
                    
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
    node = EffectsNode('play')

    # Spin the node until interrupted.
    rclpy.spin(node)

    # Shutdown the node and ROS.
    node.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()