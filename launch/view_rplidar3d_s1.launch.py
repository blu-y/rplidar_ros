#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    channel_type =  LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    arduino_port = LaunchConfiguration('arduino_port', default='/dev/ttyACM0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='115200')
    arduino_baudrate = LaunchConfiguration('arduino_baudrate', default='/dev/ttyACM0')
    stepper_rpm = LaunchConfiguration('stepper_rpm', default='2')
    frame_id = LaunchConfiguration('frame_id', default='laser')
    frame_id_3d = LaunchConfiguration('frame_id_3d', default='laser_3d')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='Sensitivity')

    rviz_config_dir = os.path.join(
            get_package_share_directory('rplidar_ros'),
            'rviz',
            'rplidar_ros.rviz')

    
    return LaunchDescription([

        DeclareLaunchArgument(
            'channel_type',
            default_value=channel_type,
            description='Specifying channel type of lidar'),
        
        DeclareLaunchArgument(
            'serial_port',
            default_value=serial_port,
            description='Specifying usb port to connected lidar'),
        
        DeclareLaunchArgument(
            'arduino_port',
            default_value=arduino_port,
            description='Specifying usb port to connected arduino'),

        DeclareLaunchArgument(
            'serial_baudrate',
            default_value=serial_baudrate,
            description='Specifying usb port baudrate to connected lidar'),

        DeclareLaunchArgument(
            'arduino_baudrate',
            default_value=arduino_baudrate,
            description='Specifying usb port baudrate to connected arduino'),

        DeclareLaunchArgument(
            'stepper_rpm',
            default_value=stepper_rpm,
            description='Specifying motor rpm to connected stepper motor'),
        
        DeclareLaunchArgument(
            'frame_id',
            default_value=frame_id,
            description='Specifying frame_id of lidar'),
        
        DeclareLaunchArgument(
            'frame_id_3d',
            default_value=frame_id_3d,
            description='Specifying frame_id of 3d pointcloud'),

        DeclareLaunchArgument(
            'inverted',
            default_value=inverted,
            description='Specifying whether or not to invert scan data'),

        DeclareLaunchArgument(
            'angle_compensate',
            default_value=angle_compensate,
            description='Specifying whether or not to enable angle_compensate of scan data'),

        DeclareLaunchArgument(
            'scan_mode',
            default_value=scan_mode,
            description='Specifying scan mode of lidar'),

        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            parameters=[{'channel_type':channel_type,
                         'serial_port': serial_port,
                         'serial_baudrate': serial_baudrate,
                         'frame_id': frame_id,
                         'inverted': inverted,
                         'angle_compensate': angle_compensate}],
            output='screen'),

        Node(
            package='rplidar_ros',
            executable='rplidar_3d',
            name='rplidar_3d',
            parameters=[{'arduino_port': arduino_port,
                         'arduino_baudrate': arduino_baudrate,
                         'stepper_rpm': stepper_rpm,
                         'frame_id_3d': frame_id_3d
                         }],
            output='screen'),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            output='screen'),
    ])

