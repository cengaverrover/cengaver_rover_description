# Cengaver Rover Description

This package contains the URDF (Unified Robot Description Format) files and launch files for the Cengaver Rover, a mobile robot simulation using ROS 2 and Gazebo.

## Installation

1. Clone the repository into your ROS 2 workspace:
    ```sh
    mkdir -p ros2_ws/src
    cd ros2_ws/src
    git clone <repository_url>
    git clone https://github.com/alpertng02/rover_teleoperation.git
    git clone https://github.com/alpertng02/mobility_controller.git
    git clone https://github.com/alpertng02/manipulator_controller.git
    git clone --recurse-submodules https://github.com/cengaverrover/bno055_ros2.git
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

### Launch Options:

You can use the following launch options to customize the launch files:

- `use_rviz:=true/false` - Enable or disable RViz. Default = false.
- `use_foxglove:=true/false` - Enable or disable Foxglove bridge. Default = true.
- `use_control_usb:=true/false` - Enable or disable control through usb. Default = true.
- `use_ds4:=true/false` - Enable or disable Duelshock 4 joy receiver. Default = false.
- `use_bno055:=true/false` - Enable or disable BNO055. Default = false.
- `use_twist_mux:=true/false` - Enable or disable Twist Mux. Default = false.
- `use_sim_time:=true/false` - Enable or disable sim_time. Default = false for rsp, true for sim launch files.

### Launch Commands:

#### Launch rover 

To launch the `rover` for the Cengaver Rover, use the following command:

```sh
ros2 launch cengaver_rover_description rover.launch.py
```

#### Launch robot_state_publisher

To launch the `robot_state_publisher` for the Cengaver Rover, use the following command:

```sh
ros2 launch cengaver_rover_description rsp.launch.py
```


#### Launch simulation

To launch the Cengaver Rover simulation in Gazebo and rviz, use the following command:
```sh
ros2 launch cengaver_rover_description launch_sim.launch.py
```

#### Launch slam simulation 
```sh
ros2 launch cengaver_rover_description slam_sim.launch.py
```

