#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Quaternion, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
import tf2_ros
import tf_transformations  # sudo apt-get install ros-foxy-tf-transformations
import math

class OdomPublisherNode(Node):
    def __init__(self):
        super().__init__('odom_publisher_node')

        ####################################
        # Physical parameters of the Robot #
        ####################################
        
        # Get robot ID parameter
        self.declare_parameter('ID', 2)
        self.ID = self.get_parameter('ID').get_parameter_value().integer_value
        
        self.wheel_base = 12.5  # Distance between wheels in cm MODIFY THIS WITH THE CHASSIS

        # Initial robot status
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # Orientation in radians

        self.v_left = 0.0
        self.v_right = 0.0

        self.last_time = self.get_clock().now().nanoseconds / 1e9

        # Subscriptions
        self.create_subscription(Float32, f'/jetros{self.ID}/speed_R', self.right_callback, 2)
        self.create_subscription(Float32, f'/jetros{self.ID}/speed_L', self.left_callback, 2)

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, f'/jetros{self.ID}/odom', 5)
        self.velocity_read_pub = self.create_publisher(Twist, f'/jetros{self.ID}/vel_read', 5)

        # TF Broadcaster
        #self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Timer
        self.create_timer(0.05, self.update_odometry)  # 20 Hz

    def left_callback(self, msg):
        self.v_left = msg.data

    def right_callback(self, msg):
        self.v_right = msg.data

    def update_odometry(self):
        current_time = self.get_clock().now().nanoseconds / 1e9
        dt = current_time - self.last_time
        self.last_time = current_time

        # Compute linear and angular velocities
        v = (self.v_right + self.v_left) / 2.0
        omega = (self.v_right - self.v_left) / self.wheel_base

        # Integrate to update position and orientation
        delta_x = v * math.cos(self.theta) * dt
        delta_y = v * math.sin(self.theta) * dt
        delta_theta = omega * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # Normalize theta to [-pi, pi]
        #self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        self.publish_odometry(v, omega)
        self.publish_velocity(v)

    def publish_odometry(self, v, omega):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        q = tf_transformations.quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = omega

        self.odom_pub.publish(odom)

        # Broadcast the transform
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = odom.pose.pose.orientation
        #self.tf_broadcaster.sendTransform(t)

    def publish_velocity(self, v):
        #Publishes the linear velocity and theta orientation.
        twist_msg = Twist()
        twist_msg.linear.x = v
        #twist_msg.angular.z = self.theta  # En este nodo, angular.z es orientación absoluta (theta)

        theta_deg = math.degrees(self.theta)
        twist_msg.angular.z = theta_deg
        self.velocity_read_pub.publish(twist_msg)
        #self.get_logger().info(f"[Velocity Read] Linear={v:.2f} m/s, Theta={theta_deg:.2f}°")


def main(args=None):
    rclpy.init(args=args)
    node = OdomPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



