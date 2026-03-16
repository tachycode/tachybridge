from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    rmw_arg = DeclareLaunchArgument(
        "rmw",
        default_value="rmw_zenoh_cpp",
        description="RMW implementation (rmw_zenoh_cpp or rmw_cyclonedds_cpp)",
    )

    set_rmw = SetEnvironmentVariable(
        name="RMW_IMPLEMENTATION",
        value=LaunchConfiguration("rmw"),
    )

    zmq_bridge_node = LifecycleNode(
        package="cpp_zmq_server",
        executable="zmq_bridge_node",
        name="zmq_bridge_node",
        namespace="",
        output="screen",
        parameters=[
            {
                "port": 5555,
                "address": "127.0.0.1",
                "work_pool_threads": 4,
                "heartbeat_timeout_s": 30,
            }
        ],
    )

    return LaunchDescription(
        [
            rmw_arg,
            set_rmw,
            zmq_bridge_node,
        ]
    )
