import os
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    rmw_arg = DeclareLaunchArgument(
        "rmw",
        default_value=os.environ.get("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp"),
        description="RMW implementation (rmw_cyclonedds_cpp or rmw_zenoh_cpp)",
    )

    set_rmw = SetEnvironmentVariable(
        name="RMW_IMPLEMENTATION",
        value=LaunchConfiguration("rmw"),
    )

    cpp_rosbridge_node = LifecycleNode(
        package="cpp_rosbridge_server",
        executable="rosbridge_server_node",
        name="cpp_rosbridge_server",
        namespace="",
        output="screen",
        parameters=[
            {
                "port": 9091,
                "address": "127.0.0.1",
                "num_threads": 4,
            }
        ],
    )
    
    habilis_communicator_node = Node(
        package="habilis_communicator",
        executable="habilis_communicator",
        name="habilis_communicator",
        output="screen",
    )

    nginx_process = ExecuteProcess(
        cmd=["bash", "-lc", "nginx -s stop >/dev/null 2>&1 || true; nginx -t && nginx -g 'daemon off;'"],
        output="screen",
    )

    return LaunchDescription(
        [
            rmw_arg,
            set_rmw,
            cpp_rosbridge_node,
            habilis_communicator_node,
            nginx_process,
        ]
    )
