from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    turtlesim_launch_path = os.path.join(
        get_package_share_directory('goal_seek'),'launch','turtlesim.launch.py'
    )

    turtlesim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(turtlesim_launch_path)
    )

    return LaunchDescription([
        turtlesim_launch,
        ############################

        # Include the node to launch here

        #############################

    ])