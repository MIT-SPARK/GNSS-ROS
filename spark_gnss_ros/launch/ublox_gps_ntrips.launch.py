import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch.actions import EmitEvent, IncludeLaunchDescription, RegisterEventHandler
import launch_ros.actions
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    config_directory = os.path.join(
        get_package_share_directory("spark_gnss_ros"), "config"
    )
    params = os.path.join(config_directory, "zed_f9p.yaml")
    ublox_gps_node = launch_ros.actions.Node(
        package="ublox_gps",
        executable="ublox_gps_node",
        output="screen",
        parameters=[params],
    )

    return LaunchDescription(
        [
            ublox_gps_node,
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=ublox_gps_node,
                    on_exit=[EmitEvent(event=Shutdown())],
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        get_package_share_directory("ntrip_client"),
                        "/launch/ntrip_client_launch.py",
                    ]
                ),
                launch_arguments={
                    "camera_model": "zed2i",
                }.items(),
            ),
        ]
    )
