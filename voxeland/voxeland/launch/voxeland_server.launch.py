import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,SetLaunchConfiguration,IncludeLaunchDescription,SetEnvironmentVariable,OpaqueFunction,GroupAction
from launch.launch_description_sources import FrontendLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
from launch.frontend.parse_substitution import parse_substitution

#===========================
def launch_arguments():
    return [
        DeclareLaunchArgument("fixed_frame", default_value="map"),
        DeclareLaunchArgument("resolution", default_value="0.02"),

        DeclareLaunchArgument("pHit", default_value="0.6"),                 # p (occupancy | hit)
        DeclareLaunchArgument("pMiss", default_value="0.4"),                # p (occupancy | miss)
        DeclareLaunchArgument("clampOccupancyMin", default_value="0.12"),   # value at which the occupancy prob gets clamped
        DeclareLaunchArgument("clampOccupancyMax", default_value="0.97"),   # value at which the occupancy prob gets clamped
   ]
#==========================

def launch_setup(context, *args, **kwargs):
    node = Node(
        package="voxeland",
        executable="voxeland_server_node",
        prefix ="xterm -hold -e ",
        parameters=[
           {"frame_id":parse_substitution("$(var fixed_frame)")},
           {"resolution":parse_substitution("$(var resolution)")},
           {"semantics_as_instances": False},
           {"sensor_model.max_range": 4.0},
           
           {"sensor_model.hit": parse_substitution("$(var pHit)")},
           {"sensor_model.miss": parse_substitution("$(var pMiss)")},
           {"sensor_model.min": parse_substitution("$(var clampOccupancyMin)")},
           {"sensor_model.max": parse_substitution("$(var clampOccupancyMax)")},
        ],
    )
    return [
        node
    ]


def generate_launch_description():

    launch_description = [
       # Set env var to print messages to stdout immediately
        SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
        SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1"),
   ]
   
    launch_description.extend(launch_arguments())
    launch_description.append(OpaqueFunction(function=launch_setup))
   
    return  LaunchDescription(launch_description)