import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import AppendEnvironmentVariable
from launch_ros.actions import Node



def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='cengaver_rover_description' #<--- CHANGE ME

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'false'}.items()
    )

    #joystick = IncludeLaunchDescription(
    #            PythonLaunchDescriptionSource([os.path.join(
    #                get_package_share_directory(package_name),'launch','joystick.launch.py'
    #            )]), launch_arguments={'use_sim_time': 'true'}.items()
    #)

    #twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    #twist_mux = Node(
    #        package="twist_mux",
    #        executable="twist_mux",
    #        parameters=[twist_mux_params, {'use_sim_time': True}],
    #        remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
    #    )

    gazebo_params_file = os.path.join(get_package_share_directory(package_name),'config','gazebo_params.yaml')
    world = os.path.join(get_package_share_directory(package_name),'worlds','fortress_museum.sdf')

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gz_server = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
                    launch_arguments={'gz_args': ['-r -s -v4 '], 'on_exit_shutdown': 'true'}.items()
             )  
    
    gz_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v4 '}.items()
    )
    
    
    set_env_vars_resources = AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(get_package_share_directory(package_name),
                        'models'))
    
    
    start_gazebo_ros_spawner_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'cengaver_rover',
            '-topic', 'robot_description',
            '-x', '0',
            '-y', '0',
            '-z', '0.01'
        ],
        output='screen',
    )
    
    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    # spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
    #                     arguments=['-topic', 'robot_description',
    #                                '-entity', 'cengaver'],
    #                     output='screen')
    
    # diff_drive_spawner = Node(
    #         package="controller_manager",
    #         executable="spawner",
    #         arguments=["diff_cont"],
    #         remappings=[('/diff_cont/cmd_vel_unstamped', '/cmd_vel')]
    #     )

    # joint_broad_spawner = Node(
    #         package="controller_manager",
    #         executable="spawner",
    #         arguments=["joint_broad"],
    #     )


    # Code for delaying a node (I haven't tested how effective it is)
    # 
    # First add the below lines to imports
    # from launch.actions import RegisterEventHandler
    # from launch.event_handlers import OnProcessExit
    #
    # Then add the following below the current diff_drive_spawner
    # delayed_diff_drive_spawner = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=spawn_entity,
    #         on_exit=[diff_drive_spawner],
    #     )
    # )
    #
    # Replace the diff_drive_spawner in the final return with delayed_diff_drive_spawner

    # Launch them all!
    return LaunchDescription([
        rsp,
        gz_server,
        gz_client,
        set_env_vars_resources,
        start_gazebo_ros_spawner_cmd,
    ])