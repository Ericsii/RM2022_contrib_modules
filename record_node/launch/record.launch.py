import ament_index_python
import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    param_path = os.path.join(get_package_share_directory('record_node'), 'record.yaml')
    record_node = launch_ros.actions.Node(
        package='record_node',
        executable="video_record",
        name='record_node',
        parameters=[param_path],
        output='screen'
    )

    ld = launch.LaunchDescription()
    ld.add_action(record_node)

    return ld