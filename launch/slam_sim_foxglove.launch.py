
import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node



def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='cengaver_rover_description' #<--- CHANGE ME

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','description_foxglove.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )
    
    slam = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('slam_toolbox'),'launch','online_async_launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'slam_params_file' : os.path.join(
                    get_package_share_directory(package_name), 'config', 'mapper_params_online_async.yaml')}.items()
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]), launch_arguments={'world' : os.path.join(
                    get_package_share_directory(package_name), 'worlds', 'cengaver_museum.xml')}.items()
             )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'cengaver_rover'],
                        output='screen')



    # Launch them all!
    return LaunchDescription([
        rsp,
        slam,
        gazebo,
        spawn_entity,
    ])
