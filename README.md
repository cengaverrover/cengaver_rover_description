# Cengaver Rover Description

This package contains the URDF (Unified Robot Description Format) files and launch files for the Cengaver Rover, a mobile robot simulation using ROS 2 and Gazebo.

## Installation

1. Clone the repository into your ROS 2 workspace:
    ```sh
    mkdir -p ros2_ws/src
    cd ros2_ws/src
    git clone <repository_url>
    ```

2. Build the package:
    ```sh
    cd ../..
    sudo apt update && rosdep update
    rosdep install --from-path src --ignore-src -y
    colcon build
    ```

3. Source the workspace:
    ```sh
    source ~/ros2_ws/install/setup.bash
    ```

## Usage

### Launch only the robot_state_publisher for the urdf

To launch robot_state_publisher for Cengaver Rover without joint_state_publisher, use the following command:

```sh
ros2 launch cengaver_rover_description rsp.launch.py
```

### Launch simulation

To launch the Cengaver Rover simulation in Gazebo and rviz, use the following command:
```sh
ros2 launch cengaver_rover_description launch_sim.launch.py
```

### Launch slam simulation 
```sh
ros2 launch cengaver_rover_description slam_sim.launch.py
```



