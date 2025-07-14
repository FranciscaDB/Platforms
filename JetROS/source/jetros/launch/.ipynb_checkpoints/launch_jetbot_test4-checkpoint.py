from launch import LaunchDescription
from launch_ros.actions import Node

vel_def = 0.0

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='jetbot_test4',
            executable='motor_controller_node',
            name='motor_controller'
        ),
        Node(
            package='jetbot_test4',
            executable='encoder_reader_left_node',
            name='encoder_reader_left_node'
        ),
        Node(
            package='jetbot_test4',
            executable='encoder_reader_right_node',
            name='encoder_reader_right_node'
        ),
        Node(
            package='jetbot_test4',
            executable='odom_publisher_node',
            name='odom_publisher'
        ),
        Node(
            package='jetbot_test4',
            executable='pid_node',
            name='PID'
        ),
        #Node(
        #    package='usb_cam',
        #    executable='usb_cam_node_exe',
        #    name='camera_node'
        #),
        #Node(
        #    package='jetbot_test4',
        #    executable='steering_node',
        #    name='steering_node',
        #    parameters=[{
        #    "vel": vel_def
        #    }]
        #),
    ])

# ros2 launch jetbotv2 launch_JetBotV2.py
