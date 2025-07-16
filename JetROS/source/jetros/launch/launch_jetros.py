#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

MIN_ANGLE = 0.78539 # pi/4
MAX_ANGLE = 3.1416 # pi
MIN_DISTANCE = 0.12
MAX_DISTANCE = 0.40
ID = 2
#vel_def = 0.0

def generate_launch_description():
    channel_type =  LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='460800')
    frame_id = LaunchConfiguration('frame_id', default='laser')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='Standard')

    filter = Node(
        package='jetros',
        executable='filter',
        name=f'jetros{ID}_lidar_filter',
        parameters=[{
            "min_angle": MIN_ANGLE,
            "max_angle": MAX_ANGLE,
            "min_distance": MIN_DISTANCE,
            "max_distance": MAX_DISTANCE,
            'ID': ID
        }]
    )
    
    motor_controller = Node(
            package='jetros',
            executable='motor_controller_node',
            name=f'jetros{ID}_motor_controller',
            parameters=[{
                'ID': ID
            }]
    )
    
    encoder_reader_left = Node(
            package='jetros',
            executable='encoder_reader_left_node',
            name=f'jetros{ID}_encoder_L',
            parameters=[{
                'ID': ID
            }]
    )

    encoder_reader_right = Node(
            package='jetros',
            executable='encoder_reader_right_node',
            name=f'jetros{ID}_encoder_R',
            parameters=[{
                'ID': ID
            }]
    )

    odom_publisher = Node(
            package='jetros',
            executable='odom_publisher_node',
            name=f'jetros{ID}_odometry',
            parameters=[{
                'ID': ID
            }]
    )

    action_prioritizer = Node(
            package='jetros',
            executable='action_prioritizer_node',
            name=f'jetros{ID}_action_prioritizer',
            parameters=[{
                'ID': ID
            }]
    )

    pid = Node(
            package='jetros',
            executable='pid_node',
            name=f'jetros{ID}_PID_Controller',
            parameters=[{
                'ID': ID
            }]
    )
    
    usb_camera = Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name=f'jetros{ID}_camera',
            parameters=[{
                'ID': ID
            }]
    )
    
    steering = Node(
            package='jetros',
            executable='steering_node',
            name=f'jetros{ID}_neural_network',
            parameters=[{
                'ID': ID
            }]
    )

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
            'serial_baudrate',
            default_value=serial_baudrate,
            description='Specifying usb port baudrate to connected lidar'),
        
        DeclareLaunchArgument(
            'frame_id',
            default_value=frame_id,
            description='Specifying frame_id of lidar'),

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
            name=f'jetros{ID}_rplidar_node',
            parameters=[{'channel_type':channel_type,
                         'serial_port': serial_port,
                         'serial_baudrate': serial_baudrate,
                         'frame_id': frame_id,
                         'inverted': inverted,
                         'angle_compensate': angle_compensate,
                         'scan_mode': scan_mode,
                         'ID': ID
                        }],
            output='screen'),
        filter,
        motor_controller,
        encoder_reader_left,
        encoder_reader_right,
        odom_publisher,
        action_prioritizer,
        pid,
        usb_camera,
        steering   
    ])


#############################3


# ros2 launch jetros launch_jetros.py
