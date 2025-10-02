import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources  import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

import xacro



def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='cengaver_rover_description' #<--- CHANGE ME
    pkg_path = os.path.join(get_package_share_directory(package_name))

    use_control_usb = LaunchConfiguration('use_control_usb')
    use_usb_cam = LaunchConfiguration('use_usb_cam')

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_ros2_control': 'false'}.items()
    )

    control = Node(
            condition=IfCondition(use_control_usb),
            package='cengaver_rover_description',
            executable='mobility_control',
            name='mobility_control',
            output='screen',
            parameters=[os.path.join(pkg_path, 'config', 'mobility_control_usb.yaml')],
    )
    
    usb_cam = Node(
            condition=IfCondition(use_usb_cam),
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam_node',
            output='screen',
            parameters=[os.path.join(pkg_path, 'config', 'usb_camera_params.yaml')],
    )


    # Launch them all!
    return LaunchDescription([
        rsp,
        control,
        usb_cam
    ])