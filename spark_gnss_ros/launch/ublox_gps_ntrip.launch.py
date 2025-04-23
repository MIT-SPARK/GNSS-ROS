import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
import launch_ros.actions
from launch.launch_description_sources import PythonLaunchDescriptionSource


def launch_setup(context, *args, **kwargs):

    config_directory = os.path.join(
        get_package_share_directory("spark_gnss_ros"), "config"
    )
    params = os.path.join(config_directory, "zed_f9p.yaml")
    
    ublox_gps_node = launch_ros.actions.Node(
        package="ublox_gps",
        executable="ublox_gps_node",
        output="screen",
        parameters=[
            params,
            ("device", LaunchConfiguration("device")),
        ],
        remappings=[
            ("~/fix", "/fix"),
            ("~/fix_velocity", "/fix_velocity"),
            ("~/navpvt", "/navpvt"),
        ]
    )

    event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=ublox_gps_node,
            on_exit=[EmitEvent(event=Shutdown())],
        )
    )

    ntrip_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                get_package_share_directory("ntrip_client"),
                "/ntrip_client_launch.py",
            ]
        ),
        launch_arguments={
            "host": LaunchConfiguration("host"),
            "port": LaunchConfiguration("port"),
            "mountpoint": LaunchConfiguration("mountpoint"),
            "username": LaunchConfiguration("username"),
            "password": LaunchConfiguration("password"),
        }.items(),
    )
    return [
        ublox_gps_node,
        event_handler,
        ntrip_ld,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "device", default_value="/dev/ttyACM0"
            ),
            DeclareLaunchArgument(
                "host", default_value="macorsrtk.massdot.state.ma.us"
            ),
            DeclareLaunchArgument(
                "port", default_value="10000"
            ),
            DeclareLaunchArgument(
                "mountpoint", default_value="RTCM3_NEAR"
            ),
            DeclareLaunchArgument("username"),
            DeclareLaunchArgument("password"),
            OpaqueFunction(function=launch_setup),
        ]
    )
