#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from Adafruit_MotorHAT import Adafruit_MotorHAT
import atexit

class MotorDriverNode(Node):
    def __init__(self):
        """
        Initializes the motor driver node, sets up motor HAT and subscribes to motor commands.
        """
        super().__init__('motor_controller_node')

        #Get robot ID parameter
        self.declare_parameter('ID', 2)
        self.ID = self.get_parameter('ID').get_parameter_value().integer_value

        ####################################
        # MotorHAT Configuration
        ####################################
        I2C_BUS = 7
        I2C_ADDR = 0x70
        self.motor_hat = Adafruit_MotorHAT(addr=I2C_ADDR, i2c_bus=I2C_BUS)

        # Assign left and right motors (matching M2 and M1)
        self.left_motor = self.motor_hat.getMotor(2)
        self.right_motor = self.motor_hat.getMotor(1)

        # Ensure motors stop on exit
        atexit.register(self.stop_motors)

        # Subscribe to velocity command topic
        self.subscription = self.create_subscription(
            Twist,
            f'/jetros{self.ID}/motors',
            self.cmd_vel_callback,
            2
        )

        #self.get_logger().info(f"motor_controller_node ready. Listening on /jetros{ID}/motors")

    def cmd_vel_callback(self, msg):
        """
        Callback function to process Twist messages from the topic.
        Converts linear and angular velocity to motor PWM values.
        """
        linear = msg.linear.x
        angular = msg.angular.z

        pwm_left = linear - angular
        pwm_right = linear + angular

        # Clamp PWM values for safety
        pwm_left = max(min(pwm_left, 255), -255)
        pwm_right = max(min(pwm_right, 255), -255)

        self.set_motor_speed(self.left_motor, pwm_left)
        self.set_motor_speed(self.right_motor, pwm_right)

    def set_motor_speed(self, motor, pwm):
        """
        Sets speed and direction for a motor based on PWM value.
        """
        direction = Adafruit_MotorHAT.FORWARD if pwm >= 0 else Adafruit_MotorHAT.BACKWARD
        motor.setSpeed(abs(int(pwm)))
        motor.run(direction)

    def stop_motors(self):
        """
        Stops both motors by releasing them.
        """
        self.left_motor.run(Adafruit_MotorHAT.RELEASE)
        self.right_motor.run(Adafruit_MotorHAT.RELEASE)
        self.get_logger().info("Motors stopped.")

def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("MotorDriverNode interrupted by user.")
    finally:
        node.stop_motors()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

