import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    config_arg = DeclareLaunchArgument(
        'config',
        default_value=os.path.join(pkg_dir, 'config', 'default.json'),
        description='Gateway config JSON file'
    )

    mcap_arg = DeclareLaunchArgument(
        'mcap_file',
        default_value='',
        description='MCAP file path for reader'
    )

    room_arg = DeclareLaunchArgument(
        'room_id',
        default_value='01',
        description='Room ID for namespace isolation'
    )

    gateway = ExecuteProcess(
        cmd=['ros2', 'run', 'cpp_zenoh_gateway', 'zenoh_gateway_node',
             LaunchConfiguration('config')],
        output='screen'
    )

    reader = ExecuteProcess(
        cmd=['ros2', 'run', 'cpp_mcap_reader', 'mcap_reader_node',
             LaunchConfiguration('mcap_file'),
             LaunchConfiguration('room_id')],
        output='screen'
    )

    return LaunchDescription([
        config_arg, mcap_arg, room_arg,
        gateway, reader
    ])
