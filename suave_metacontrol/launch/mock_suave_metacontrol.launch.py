import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    pkg_suave_path = get_package_share_directory(
        'suave')

    mros2_system_modes_bridge_node = Node(
        package='mros2_reasoner',
        executable='mros2_system_modes_bridge',
    )

    mock_launch_path = os.path.join(
        pkg_suave_path,
        'launch',
        'mock.launch.py')

    mock_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mock_launch_path),
    )

    return LaunchDescription([
        mock_launch,
        mros2_system_modes_bridge_node,
    ])
