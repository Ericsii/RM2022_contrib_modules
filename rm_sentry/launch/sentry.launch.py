import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace

def generate_launch_description():

    param_path = os.path.join(get_package_share_directory("rm_sentry"), "config/rm_sentry.yaml")
    robot_name = 'sentry'
    with open(param_path, 'r') as f:
        params = yaml.safe_load(f)['sentry']['ros__parameters']
    auto_aim_node = Node(
        package="rm_sentry",
        executable="sentry_node",
        namespace = robot_name,
        name="sentry_node",
        parameters=[params],
        output='screen'
    )
    print(params)
    ld = LaunchDescription()
    ld.add_action(auto_aim_node)

    return ld