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
   ]
#==========================

def launch_setup(context, *args, **kwargs):
    node = Node(
        package="voxeland",
        executable="voxeland_server_node",
        prefix ="xterm -hold -e",
        parameters=[
           {"frame_id":parse_substitution("$(var fixed_frame)")},
           {"resolution":parse_substitution("$(var resolution)")},
           {"semantics_as_instances": False},
           {"sensor_model.max_range": 4.0},
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