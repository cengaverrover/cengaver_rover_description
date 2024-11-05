# Cengaver Rover Description

This package contains the URDF (Unified Robot Description Format) files and launch files for the Cengaver Rover, a mobile robot simulation using ROS 2 and Gazebo.

## Installation

1. Clone the repository into your ROS 2 workspace:
    ```sh
    cd ~/ros2_ws/src
    git clone <repository_url>
    ```

2. Build the package:
    ```sh
    cd ~/ros2_ws
    colcon build
    ```

3. Source the workspace:
    ```sh
    source ~/ros2_ws/install/setup.bash
    ```

## Usage

### Launch the rviz visualizer

To launch robot_state_publisher for Cengaver Rover without joint_state_publisher, use the following command:

```sh
ros2 launch cengaver_rover_description cengaver_rover_description.launch.py
```

### Launch the rviz visualizer

To view the Cengaver Rover in rviz with joint_state_publisher_gui, use the following command:

```sh
ros2 launch cengaver_rover_description cengaver_rover_rviz.launch.py
```
### Launch the Simulation

To launch the Cengaver Rover simulation in Gazebo, use the following command:

```sh
ros2 launch cengaver_rover_description launch_sim.launch.py
```
### Launch robot_state_publisher with foxglove_bridge
To launch Cengaver Rover robot_state_publisher with foxglove_bridge, use the following command:

```sh
ros2 launch cengaver_rover_description cengaver_rover_description_foxglove.launch.py
```

