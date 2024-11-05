#!/usr/bin/env python3 import rospy
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node



def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='cengaver_rover_description' #<--- CHANGE ME

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','cengaver_rover_description.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Include the Foxglove launch file, provided by the foxglove_bridge package
    # Include the Foxglove launch file, provided by the foxglove_bridge package
    foxglove = IncludeLaunchDescription(
            XMLLaunchDescriptionSource([os.path.join(
                get_package_share_directory('foxglove_bridge'), 'launch', 'foxglove_bridge_launch.xml')]),
         )

    # Launch them all!
    return LaunchDescription([
        rsp,
        foxglove
    ])
