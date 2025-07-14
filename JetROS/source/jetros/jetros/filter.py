#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32
import numpy as np
import math

class FilteredLidar(Node):
    def __init__(self):
        super().__init__('filtered_lidar_node')

        # Get robot ID parameter
        self.declare_parameter('ID', 2)
        self.ID = self.get_parameter('ID').get_parameter_value().integer_value

        # Subscribe to the raw LaserScan topic for this robot
        self.subscription = self.create_subscription(
            LaserScan,
            f'/jetros{self.ID}/scan',
            self.scan_callback,
            100)

        # Publishers for filtered scan, object detection, and closest distance
        self.publisher = self.create_publisher(
            LaserScan,
            f'/jetros{self.ID}/filtered_scan',
            100)
        self.object_detected_pub = self.create_publisher(
            Bool,
            f'/jetros{self.ID}/obj_det',
            2)
        self.closest_distance_pub = self.create_publisher(
            Float32,
            f'/jetros{self.ID}/dist_read',
            2)

        # Filtering parameters (angle in radians, distance in meters)
        self.declare_parameter('min_angle', -1.5)  # Rango de -30° a 30° (en radianes)
        self.declare_parameter('max_angle', 1.5)
        self.declare_parameter('min_distance', 0.05)  # Min 5cm
        self.declare_parameter('max_distance', 1.0)  # Max 1m
    

    def scan_callback(self, msg):
        # Retrieve filtering parameters
        min_angle = self.get_parameter('min_angle').get_parameter_value().double_value
        max_angle = self.get_parameter('max_angle').get_parameter_value().double_value
        min_distance = self.get_parameter('min_distance').get_parameter_value().double_value
        max_distance = self.get_parameter('max_distance').get_parameter_value().double_value

        # Create a copy of the incoming scan message
        filtered_msg = LaserScan()
        filtered_msg.header = msg.header
        filtered_msg.header.frame_id = 'laser'
        filtered_msg.header.stamp = self.get_clock().now().to_msg()
        filtered_msg.angle_min = msg.angle_min
        filtered_msg.angle_max = msg.angle_max
        filtered_msg.angle_increment = msg.angle_increment
        filtered_msg.time_increment = msg.time_increment
        filtered_msg.scan_time = msg.scan_time
        filtered_msg.range_min = min_distance
        filtered_msg.range_max = max_distance

        # Convert index range to angles for filtering
        num_readings = len(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, num_readings)

        # Filter each laser point by angle and distance
        filtered_ranges = []
        detected_points = []
        for i in range(num_readings):
            if ((min_angle <= angles[i] <= max_angle) or (-max_angle <= angles[i] <= -min_angle)) and (min_distance <= msg.ranges[i] <= max_distance) and np.isfinite(msg.ranges[i]):
                filtered_ranges.append(msg.ranges[i])
                detected_points.append(msg.ranges[i])
            else:
                filtered_ranges.append(float('inf'))  # Discard invalid or out-of-range values

        filtered_msg.ranges = filtered_ranges
        filtered_msg.intensities = msg.intensities

        # Publish the filtered scan message
        self.publisher.publish(filtered_msg)

        # Publish object detection flag
        object_detected = Bool()
        object_detected.data = len(detected_points) > 0
        self.object_detected_pub.publish(object_detected)

        # Publish the closest detected distance or NaN if none found
        closest_distance = Float32()
        if detected_points:
            closest_distance.data = float(min(detected_points))
        else:
            closest_distance.data = float('nan')
        self.closest_distance_pub.publish(closest_distance)

        # Optional: Uncomment for debugging
        #self.get_logger().info('Publishing filtered datav of LiDAR, detection: %s, distance: %.4f ' % (object_detected.data, closest_distance.data))



def main(args=None):
    rclpy.init(args=args)
    node = FilteredLidar()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()