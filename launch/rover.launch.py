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
    use_ds4 = LaunchConfiguration('use_ds4')
    use_rplidar = LaunchConfiguration('use_rplidar')
    use_bno055 = LaunchConfiguration('use_bno055')

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_ros2_control': 'false'}.items()
    )
    
    bno055 = Node(
            condition=IfCondition(use_bno055),
            package='bno055_ros2',
            executable='bno055_publisher',
            name='bno055_node',
            output='screen',
            parameters=[os.path.join(pkg_path, 'config', 'bno055_config.yaml')],
    )

    rplidar = Node(
            condition=IfCondition(use_rplidar),
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_composition',
            output='screen',
            parameters=[os.path.join(pkg_path, 'config', 'rplidar_config.yaml')],
    )

    ds4_receiver = Node(
            condition=IfCondition(use_ds4),
            package='rover_teleoperation',
            executable='joy_receiver',
            name='joy_receiver',
            output='screen',
            parameters=[os.path.join(pkg_path, 'config', 'ds4_config.yaml')],
    )

    mobility_controller = Node(
            condition=IfCondition(use_control_usb),
            package='mobility_controller',
            executable='mobility_controller',
            name='mobility_controller',
            output='screen',
            parameters=[os.path.join(pkg_path, 'config', 'mobility_controller_usb.yaml')],
    )
    
    manipulator_controller = Node(
            condition=IfCondition(use_control_usb),
            package='manipulator_controller',
            executable='manipulator_controller',
            name='manipulator_controller',
            output='screen',
            parameters=[os.path.join(pkg_path, 'config', 'manipulator_controller_usb.yaml')],
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
        
        DeclareLaunchArgument(
            'use_control_usb',
            default_value='true',
            description='Use usb mobility controller if true'),
        DeclareLaunchArgument(
            'use_ds4',
            default_value='true',
            description='Use Duelshock 4 for joy'), 
        DeclareLaunchArgument(
            'use_usb_cam',
            default_value='false',
            description='Use usb cam if true'),
        DeclareLaunchArgument(
            'use_rplidar',
            default_value='false',
            description='Use RpLidar if true'),
        DeclareLaunchArgument(
            'use_bno055',
            default_value='false',
            description='Use BNO055 if true'),
        rsp,
        ds4_receiver,
        rplidar,
        bno055,
        mobility_controller,
        manipulator_controller,
        usb_cam
    ])