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
    
    package_name='cengaver_rover_description' #<--- CHANGE ME

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    use_foxglove_bridge = LaunchConfiguration('use_foxglove')
    use_rviz= LaunchConfiguration('use_rviz')
    use_twist_mux= LaunchConfiguration('use_twist_mux')
    use_robot_localization = LaunchConfiguration('use_robot_localization')


    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('cengaver_rover_description'))
    xacro_file = os.path.join(pkg_path,'models','cengaver.urdf.xacro')
    # robot_description_config = xacro.process_file(xacro_file).toxml()
    robot_description_config = Command(['xacro ', xacro_file, ' use_ros2_control:=', use_ros2_control, ' sim_mode:=', use_sim_time])

    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Create a foxglove_bridge
    foxglove_bridge = IncludeLaunchDescription(
            XMLLaunchDescriptionSource([os.path.join(
                get_package_share_directory('foxglove_bridge'), 'launch', 'foxglove_bridge_launch.xml')]),
            condition=IfCondition(use_foxglove_bridge),
            launch_arguments={'use_sim_time': use_sim_time, 'max_qos_depth': '400'}.items()
    )

    
    robot_localization = Node(
            condition=IfCondition(use_robot_localization),
            package='robot_localization',
            executable='ukf_node',
            name='ukf_filter_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}, os.path.join(pkg_path, 'config', 'ukf.yaml')],
    )

    rviz_config_path = os.path.join(pkg_path, 'config', 'urdf_config.rviz')
    rviz = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', rviz_config_path]
    )

    laser_filter_params = {'input_topic': 'scan_raw', 'output_topic': 'scan', 'filtered_ranges': [-0.611, 0.611], 'use_sim_time': use_sim_time}
    laser_filter = Node(
        package='cengaver_rover_description',
        executable='laser_filter',
        output='screen',
        parameters=[laser_filter_params]
    )
    
    twist_mux_params = {'use_sim_time': use_sim_time}
    twist_mux = Node(
        condition=IfCondition(use_twist_mux),
        package='twist_mux',
        executable='twist_mux',
        output='screen',
        parameters=[twist_mux_params],
        arguments=['--params-file', os.path.join(
                    pkg_path, 'config', 'twist_mux.yaml')] 
    )

    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        DeclareLaunchArgument(
            'use_ros2_control',
            default_value='false',
            description='Use ros2_control if true'),
        DeclareLaunchArgument(
           'use_foxglove',
           default_value='true',
           description='Use foxglove_bridge if true'),
        DeclareLaunchArgument(
           'use_rviz',
           default_value='false',
           description='Use rviz if true'),
        DeclareLaunchArgument(
           'use_twist_mux',
           default_value='false',
           description='Use twist_mux if true'),
        DeclareLaunchArgument(
            'use_robot_localization',
            default_value='true',
            description='Use robot_localization for odom if true'),        

        node_robot_state_publisher,
        laser_filter,
        foxglove_bridge,
        twist_mux,
        robot_localization,
        rviz        

    ])