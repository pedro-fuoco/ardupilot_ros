# ardupilot_ros: ROS 2 use cases with Ardupilot

## Requirements :
* [ROS Humble](https://docs.ros.org/en/humble/Installation.html)

* [Gazebo Garden](https://gazebosim.org/docs/garden/install)

* [ardupilot_gz](https://github.com/ArduPilot/ardupilot_gz)
    * Here are two workarounds for know issues in the installation of `ardupilot_gz` (they should be fixed soon)
        * [Handle microxrceddsgen dependency](https://github.com/ArduPilot/ardupilot_gz/issues/19)
        * Set the ENABLE_DDS ardupilot parameter manually if it isn't set

## Installation

Clone this repository into your ros2 workspace alongside ardupilot_gz
```bash
cd ~/ros2_ws/src
git clone git@github.com:ardupilot/ardupilot_ros.git -b humble
```
Build it with colcon build
```bash
cd ~/ros2_ws
source install/setup.sh
colcon build --packages-select ardupilot_ros
```

## Usage

### 1. Cartographer running with lidar on copter

This simulation has an iris copter equipped with a 360 degrees 2D lidar in a maze world, to launch it run:

```bash
ros2 launch ardupilot_gz_bringup iris_maze.launch.py
```
With the world and copter in place, launch cartographer to generate SLAM:

```bash
ros2 launch ardupilot_ros cartographer.launch.py
```
