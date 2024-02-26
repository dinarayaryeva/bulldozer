from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory("bulldozer_planner"),
        "config",
        "params.yaml",
    )

    global_planner_node = Node(
        package='bulldozer_planner',
        executable='global_planner',
        name='global_planner_node',
        parameters=[config]
    )

    ld.add_action(global_planner_node)
    return ld