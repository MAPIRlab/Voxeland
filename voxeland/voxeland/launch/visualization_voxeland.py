import os
from enum import Enum
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetLaunchConfiguration, IncludeLaunchDescription, SetEnvironmentVariable, OpaqueFunction, GroupAction
from launch.launch_description_sources import FrontendLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
from launch.frontend.parse_substitution import parse_substitution
from ros2launch.api import get_share_file_path_from_package
# ===========================


def launch_arguments():
    return [
        # DeclareLaunchArgument("", default_value=""),
    ]
# ==========================

def launch_setup(context, *args, **kwargs):

    voxeland_server = Node(
        package="voxeland",
        executable="voxeland_server_node",
        name="voxeland_server",
        # prefix ="xterm -hold -e",
        parameters=[
           {"resolution":0.05},
           {"latch": False},
           {"semantics_as_instances": True},
           
           {"occupancy_min_z": 0.0},
           {"occupancy_max_z": 10.0},

           {"log_level": "Debug"},

           {"load_map_path_ply": "/mnt/HDD/Evaluation_Voxeland/SceneNN/evaluation2016/223/voxeland_pointcloud.ply"},
           {"load_map_path_json": "/mnt/HDD/Evaluation_Voxeland/SceneNN/evaluation2016/223/voxeland_instanceMap.json"},

        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        # prefix="xterm -e",
        arguments=[
            "-d" + os.path.join(get_package_share_directory(
                "semantic_scenarios"), "launch", "voxeland.rviz")
        ],
    )

    return [
        voxeland_server,
        rviz,
    ]

# ==========================


def generate_launch_description():

    launch_description = [
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
        SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1"),
    ]

    launch_description.extend(launch_arguments())
    launch_description.append(OpaqueFunction(function=launch_setup))

    return LaunchDescription(launch_description)
