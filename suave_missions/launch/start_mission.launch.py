import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import LaunchConfigurationEquals
from launch.conditions import LaunchConfigurationNotEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    result_filename = LaunchConfiguration('result_filename')

    result_filename_arg = DeclareLaunchArgument(
        'result_filename',
        default_value='',
        description='Name of the results file'
    )

    pkg_suave_path = get_package_share_directory(
        'suave_missions')

    mission_config = os.path.join(
        get_package_share_directory('suave_missions'),
        'config',
        'mission_config.yaml')

    mission_node_filename_override = Node(
        package='suave_missions',
        executable='start_mission',
        name='parent_mission_node',
        parameters=[mission_config, {
            'result_filename': result_filename,
        }],
        condition=LaunchConfigurationNotEquals('result_filename', '')
    )

    mission_node = Node(
        package='suave_missions',
        executable='start_mission',
        name='parent_mission_node',
        parameters=[mission_config],
        condition=LaunchConfigurationEquals('result_filename', '')
    )

    pkg_suave_path = get_package_share_directory(
        'suave')
    suave_launch_path = os.path.join(
        pkg_suave_path,
        'launch',
        'suave.launch.py')

    suave_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(suave_launch_path),
        condition=LaunchConfigurationNotEquals(
            'adaptation_manager', 'metacontrol'))

    pkg_suave_metacontrol_path = get_package_share_directory(
        'suave_metacontrol')

    metacontrol_launch_path = os.path.join(
        pkg_suave_metacontrol_path,
        'launch',
        'suave_metacontrol.launch.py')

    metacontrol_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(metacontrol_launch_path),
        condition=LaunchConfigurationEquals(
            'adaptation_manager', 'metacontrol'),)

    random_launch_path = os.path.join(
        pkg_suave_metacontrol_path,
        'launch',
        'suave_random.launch.py')

    random_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(random_launch_path),
        condition=LaunchConfigurationEquals('adaptation_manager', 'random'))

    return LaunchDescription([
        result_filename_arg,
        mission_node,
        mission_node_filename_override,
        metacontrol_launch,
        suave_launch,
        random_launch,
    ])
