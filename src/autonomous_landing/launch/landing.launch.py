"""Bring up the landing stack: PX4 XRCE-DDS agent, AprilTag detector, offboard controller.

Isaac Sim + Pegasus (PX4 SITL) must already be running.

Examples:
  ros2 launch autonomous_landing landing.launch.py
  ros2 launch autonomous_landing landing.launch.py start_control:=false
  ros2 launch autonomous_landing landing.launch.py \
      image_topic:=/drone/camera/image_raw camera_info_topic:=/drone/camera/camera_info
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('autonomous_landing')

    use_sim_time = LaunchConfiguration('use_sim_time')

    args = [
        DeclareLaunchArgument('start_agent', default_value='true',
                              description='Start MicroXRCEAgent (PX4 <-> ROS 2 bridge)'),
        DeclareLaunchArgument('agent_port', default_value='8888',
                              description='UDP port of the XRCE-DDS agent'),
        DeclareLaunchArgument('start_detector', default_value='true',
                              description='Start the apriltag_ros detector'),
        DeclareLaunchArgument('start_control', default_value='true',
                              description='Start the velocity setpoint controller'),
        DeclareLaunchArgument('image_topic', default_value='/drone1/camera/color/image_raw',
                              description='Downward camera image topic'),
        DeclareLaunchArgument('camera_info_topic', default_value='/drone1/camera/color/camera_info',
                              description='Downward camera CameraInfo topic'),
        DeclareLaunchArgument('apriltag_config',
                              default_value=os.path.join(pkg_share, 'config', 'apriltag.yaml'),
                              description='apriltag_ros parameter file'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use /clock from Isaac Sim (only if it is published)'),
    ]

    xrce_agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', LaunchConfiguration('agent_port')],
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_agent')),
    )

    # The sim stamps image and camera_info differently, so apriltag_ros (exact-time sync)
    # never gets a matching pair. Re-stamp camera_info with each image's header.
    synced_info_topic = '/apriltag/camera_info_synced'
    camera_info_sync = Node(
        package='autonomous_landing',
        executable='camera_info_sync',
        name='camera_info_sync',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('image', LaunchConfiguration('image_topic')),
            ('camera_info_in', LaunchConfiguration('camera_info_topic')),
            ('camera_info_out', synced_info_topic),
        ],
        condition=IfCondition(LaunchConfiguration('start_detector')),
    )

    apriltag = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag',
        output='screen',
        parameters=[LaunchConfiguration('apriltag_config'), {'use_sim_time': use_sim_time}],
        remappings=[
            ('image_rect', LaunchConfiguration('image_topic')),
            ('camera_info', synced_info_topic),
        ],
        condition=IfCondition(LaunchConfiguration('start_detector')),
    )

    control = Node(
        package='autonomous_landing',
        executable='velocity_setpoint_control',
        name='velocity_setpoint_control',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('start_control')),
    )

    return LaunchDescription(args + [xrce_agent, camera_info_sync, apriltag, control])
