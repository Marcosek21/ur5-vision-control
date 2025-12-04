from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Parametr: czy użyć ArUco czy klikania
    use_aruco = LaunchConfiguration('use_aruco')

    declare_use_aruco = DeclareLaunchArgument(
        'use_aruco',
        default_value='true',
        description='If true → start aruco_controller_node, else click_controller_node'
    )

    # Ścieżka do ur_robot_driver launch
    ur_launch_file = os.path.join(
        get_package_share_directory('ur_robot_driver'),
        'launch',
        'ur_control.launch.py'
    )

    # Node kamery
    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        output='screen'
    )

    # Node klikowy
    click_node = Node(
        package='ur5_vision_control',
        executable='click_controller_node',
        name='click_controller_node',
        output='screen',
        condition=IfCondition(PythonExpression(['not ', use_aruco]))
    )

    # Node ArUco
    aruco_node = Node(
        package='ur5_vision_control',
        executable='aruco_controller_node',
        name='aruco_controller_node',
        output='screen',
        condition=IfCondition(use_aruco)
    )

    # Sterownik UR5
    ur_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ur_launch_file),
        launch_arguments={
            'ur_type': 'ur5',
            'robot_ip': '192.168.56.101',
            'launch_rviz': 'True'
        }.items()
    )

    return LaunchDescription([
        declare_use_aruco,
        usb_cam_node,
        click_node,
        aruco_node,
        ur_driver
    ])
