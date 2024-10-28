"""Show the actual robot in RVIZ

   This does not create joint commands, so you can move by hand.

   This should start
     1) RVIZ, ready to view the robot
     2) The robot_state_publisher (listening to /joint_states)
     3) The HEBI node to communicate with the motors

"""

import os
import xacro

from ament_index_python.packages import get_package_share_directory as pkgdir

from launch                            import LaunchDescription
from launch.actions                    import Shutdown
from launch_ros.actions                import Node


#
# Generate the Launch Description
#
def generate_launch_description():

     ######################################################################
    
    ps5 = Node(
        name       = 'ps5', 
        package    = 'walle',
        executable = 'ps5',
        output     = 'screen')
    
    motors = Node(
        name       = 'motors', 
        package    = 'walle',
        executable = 'motors',
        output     = 'screen')
    
    servos = Node(
        name       = 'servos', 
        package    = 'walle',
        executable = 'servos',
        output     = 'screen'
    )
    
    webserver = Node(
        name       = 'webserver', 
        package    = 'walle',
        executable = 'webserver',
        output     = 'screen'
    )
    
    effects = Node(
        name       = 'effects', 
        package    = 'walle',
        executable = 'effects',
        output     = 'screen'
    )


    ######################################################################
    # COMBINE THE ELEMENTS INTO ONE LIST
    
    # Return the description, built as a python list.
    return LaunchDescription([
        ps5,
        motors,
        servos,
        webserver,
        effects
    ])