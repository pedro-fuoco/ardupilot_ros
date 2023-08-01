# Copyright 2023 ArduPilot.org.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.

# Adapted from https://github.com/gazebosim/ros_gz_project_template
#
# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch navigation2 with SLAM."""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import SetRemap
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution


from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate a launch description for the navigation example."""
    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')
    ardupilot_ros_dir = get_package_share_directory("ardupilot_ros")
    
    # Navigation
    navigation = GroupAction(
        actions=[
            SetRemap(src='/tf',dst='/ap/tf'),
            SetRemap(src='/tf_static',dst='/ap/tf_static'),
            SetRemap(src='/cmd_vel',dst='/ap/cmd_vel'),
            SetRemap(src='/scan',dst='/laser/scan'),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, 'navigation_launch.py')),
                launch_arguments={'use_sim_time': 'true',
                                  'params_file':os.path.join(FindPackageShare('ardupilot_ros').find('ardupilot_ros'), "config", "navigation.yaml")}.items())
        ]
    )  
    
    # RViz.
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            os.path.join(FindPackageShare('ardupilot_ros').find('ardupilot_ros'), "rviz", "navigation.rviz"),
        ],
        condition=IfCondition(LaunchConfiguration("rviz")),
        remappings = [
            ('/tf', '/ap/tf'),
            ('/tf_static', '/ap/tf_static'),
            ('/scan', '/laser/scan')
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz", default_value="true", description="Open RViz."
            ),
            rviz,
            navigation,
        ]
    )
