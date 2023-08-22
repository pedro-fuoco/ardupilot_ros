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

If you'd like to get the information from Cartographer to go into Ardupilot's extended kalman filter, you will need to change some parameters, you can do that through any GCS, including mavproxy:

-  AHRS_EKF_TYPE = 3 to use EKF3
-  EK2_ENABLE = 0 to disable EKF2
-  EK3_ENABLE = 1 to enable EKF3
-  EK3_SRC1_POSXY = 6 to set position horizontal source to ExternalNAV
-  EK3_SRC1_POSZ = 1 to set position vertical source to Baro
-  EK3_SRC1_VELXY = 6 to set velocity horizontal source to ExternalNAV
-  EK3_SRC1_VELZ = 6 to set vertical velocity source to ExternalNAV
-  EK3_SRC1_YAW = 6 to set yaw source to ExternalNAV
-  VISO_TYPE = 1 to enable visual odometry
-  ARMING_CHECK = 388598 (optional, to disable GPS checks)

The parameters above are recommended for SITL. If you plan on using this on a real copter, it is a good idea to setup a second source of EKF. This way the robot doesn't crash if the external odometry you are providing stops publishing or gets lost.

Please refer to this link for more information on [Common EKF Sources](https://ardupilot.org/copter/docs/common-ekf-sources.html>) as well as this guide on [GPS / Non-GPS Transitions](https://ardupilot.org/copter/docs/common-non-gps-to-gps.html).
