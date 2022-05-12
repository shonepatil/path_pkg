from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python import get_package_share_directory
import os


def generate_launch_description():
    path_pkg = 'path_pkg'
    param_file_name = 'path_provider.yaml'
    path_file_name = 'path_from_odom.csv'
    node_name = 'path_provider_node'

    ld = LaunchDescription()

    param_file = os.path.join(
        get_package_share_directory(path_pkg),
        'param',
        param_file_name)

    path_file = os.path.join(
        get_package_share_directory(path_pkg), 
        'paths', 
        path_file_name)

    path_node = Node(
        package=path_pkg,
        executable=node_name,
        output='screen',
        parameters=[param_file, {"csv_path": path_file}],
        remappings=[("/trajectory", "/global_trajectory")],
        )

    ld.add_action(path_node)
    return ld