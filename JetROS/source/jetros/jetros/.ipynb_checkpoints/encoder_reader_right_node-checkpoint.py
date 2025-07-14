#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import Jetson.GPIO as GPIO
from std_msgs.msg import Float32
import time
import threading
import math

# ---------------------- CONFIGURATION -----------------------
ENCODER_PIN1 = 11
ENCODER_PIN2 = 12
CPR = 2800  # 28 x 100 gear ratio
WHEEL_RADIUS = 2.15  # cm
PUBLICATION_INTERVAL = 0.02  # 20 ms

# ---------------------- GLOBAL VARIABLES ------------------------
pulses = 0
lastEncoded = 0

# ---------------------- ENCODER INTERRUPTS -----------------------
GPIO.setmode(GPIO.BOARD)
GPIO.setup(ENCODER_PIN1, GPIO.IN)
GPIO.setup(ENCODER_PIN2, GPIO.IN)

# Callback to handle encoder transitions
def update_encoder(channel):
    global lastEncoded, pulses
    MSB = GPIO.input(ENCODER_PIN1)
    LSB = GPIO.input(ENCODER_PIN2)
    encoded = (MSB << 1) | LSB
    transition = (lastEncoded << 2) | encoded

    # Determine direction of rotation based on transition code
    if transition in [0b1101, 0b0100, 0b0010, 0b1011]:
        pulses -= 1  # Rotating in one direction
    elif transition in [0b1110, 0b0111, 0b0001, 0b1000]:
        pulses += 1  # Rotating in the other direction

    lastEncoded = encoded

GPIO.add_event_detect(ENCODER_PIN1, GPIO.BOTH, callback=update_encoder)
GPIO.add_event_detect(ENCODER_PIN2, GPIO.BOTH, callback=update_encoder)

# ---------------------- ROS 2 NODE -----------------------
class EncoderReaderNode(Node):
    def __init__(self):
        super().__init__('encoder_reader_right_node')
        #self.get_logger().info("Right encoder node started.")

        # Get robot ID from parameter
        self.declare_parameter('ID', 2)
        self.ID = self.get_parameter('ID').get_parameter_value().integer_value

        # Publisher for right wheel linear velocity
        self.speed_R_pub = self.create_publisher(Float32, f'/jetros{self.ID}/speed_R', 2)
        self.last_pulses = 0
        self.last_time = time.time()
        self.lock = threading.Lock()

        # Timer to periodically compute and publish velocity
        self.create_timer(PUBLICATION_INTERVAL, self.timer_callback)

    def timer_callback(self):
        global pulses
        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        if dt == 0:
            return

        with self.lock:
            current_pulses = pulses
            delta = current_pulses - self.last_pulses
            self.last_pulses = current_pulses

        # Convert pulse count to linear velocity (cm/s)
        revolutions = delta / CPR
        linear_velocity = (2 * math.pi * WHEEL_RADIUS * revolutions) / dt  # cm/s

        msg = Float32()
        msg.data = linear_velocity
        self.speed_R_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = EncoderReaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Encoder node stopped by user.")
    finally:
        node.destroy_node()
        GPIO.cleanup()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

