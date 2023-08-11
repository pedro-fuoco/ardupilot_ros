from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from pathlib import Path


def generate_launch_description():
    ## ***** Launch arguments *****
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="true")
    rviz_arg = DeclareLaunchArgument(
        "rviz", default_value="true", description="Open RViz."
    )

    ## ***** Nodes *****
    cartographer_node = Node(
        package="cartographer_ros",
        executable="cartographer_node",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        arguments=[
            "-configuration_directory",
            FindPackageShare("ardupilot_ros").find("ardupilot_ros") + "/config",
            "-configuration_basename",
            "cartographer.lua",
        ],
        remappings=[
            ("/scan", "/laser/scan"),
            ("/tf", "/ap/tf"),
        ],
        output="screen",
    )

    cartographer_occupancy_grid_node = Node(
        package="cartographer_ros",
        executable="cartographer_occupancy_grid_node",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {"resolution": 0.05},
        ],
    )

    # RViz.
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            str(
                Path(
                    FindPackageShare("ardupilot_ros").find("ardupilot_ros"),
                    "rviz",
                    "cartographer.rviz",
                )
            ),
        ],
        condition=IfCondition(LaunchConfiguration("rviz")),
        remappings=[
            ("/tf", "/ap/tf"),
            ("/tf_static", "/ap/tf_static"),
        ],
    )

    return LaunchDescription(
        [
            # Arguments
            use_sim_time_arg,
            rviz_arg,
            # Nodes
            cartographer_node,
            cartographer_occupancy_grid_node,
            rviz,
        ]
    )
