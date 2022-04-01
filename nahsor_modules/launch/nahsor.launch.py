import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    param_path = os.path.join(get_package_share_directory("nahsor_modules"), "config/nahsor_param.yaml")

    nahsor_node = Node(
        package="nahsor_modules",
        executable="nahsor",
        name="nahsor",
        parameters=[param_path]
    )
    ld = LaunchDescription()
    ld.add_action(nahsor_node)

    return ld