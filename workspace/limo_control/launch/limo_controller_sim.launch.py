#!/usr/bin/python3

from os.path import join

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Simulation launch from limo_simulation package
    sim_launch_file = join(
        get_package_share_directory("limo_simulation"), "launch", "limo.launch.py"
    )
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_launch_file),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # Limo controller node
    controller_node = Node(
        package="limo_control",
        executable="limo_controller_node",
        name="limo_controller_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # Combine everything into the LaunchDescription
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time", default_value="true", description="Use simulation time"
            ),
            sim,
            controller_node,
        ]
    )
