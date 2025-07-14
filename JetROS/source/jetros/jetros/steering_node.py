import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image as ROSImage
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from cv_bridge import CvBridge

import torch
from torch2trt import TRTModule
import torchvision.transforms as transforms
import PIL.Image
import numpy as np
import time
import statistics
import atexit

class SteeringNode(Node):
    def __init__(self):
        super().__init__('steering_node')
        
        # Get robot ID parameter
        self.declare_parameter('ID', 2)
        self.ID = self.get_parameter('ID').get_parameter_value().integer_value
        #self.ns = f'jetros{self.ID}'

        # Default velocity
        self.vel = 0.0

        # Bridge for converting ROS Image to OpenCV
        self.bridge = CvBridge()

        # Load optimized model with TensorRT
        self.device = torch.device('cuda')
        self.model_trt = TRTModule()
        self.model_trt.load_state_dict(
            torch.load('/home/nelson/ros2_ws/src/jetbot_test6/jetbot_test6/best_steering_model_xy_trt.pth')
        )

        # Image preprocessing normalization values
        self.mean = torch.Tensor([0.485, 0.456, 0.406]).cuda().half()
        self.std = torch.Tensor([0.229, 0.224, 0.225]).cuda().half()

        # Subscribe to image and velocity topics
        self.image_sub = self.create_subscription(
            ROSImage,
            f'/jetros{self.ID}/image_raw',
            self.listener_callback,
            2
        )

        self.vel_sub = self.create_subscription(
            Float32,
            f'/jetros{self.ID}/vel',
            self.vel_callback,
            2
        )

        # Publishing Twist Commands
        self.twist_pub = self.create_publisher(
            Twist,
            f'/jetros{self.ID}/target',
            2
        )

        """        
        # Latencia
        self.latency_list = []
        self.latency_file = open('latencias_red_neuronal.txt', 'w')
        self.latency_file.write("latency_seconds\n")
        atexit.register(self.on_exit)

        # Timer para estadísticas cada 10 s
        self.create_timer(10.0, self.report_latency_stats)
        """
        #self.get_logger().info('Steering node initialized and waiting for /vel updates.')

    def vel_callback(self, msg):
         # Update linear velocity
        self.vel = float(msg.data)
        self.get_logger().info(f'Updated velocity: {self.vel:.2f}')

    def preprocess(self, image):
         # Resize, convert to tensor, normalize
        image = PIL.Image.fromarray(image).resize((224, 224))
        image = transforms.functional.to_tensor(image).to(self.device).half()
        image.sub_(self.mean[:, None, None]).div_(self.std[:, None, None])
        return image[None, ...]

    def listener_callback(self, msg):
        try:
            # Convert ROS image to OpenCV
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            """
            # Medir tiempo de inferencia
            start_time = time.time()
            """
            # Run inference
            image_tensor = self.preprocess(frame)
            xy = self.model_trt(image_tensor).detach().float().cpu().numpy().flatten()

            """            
            latency = time.time() - start_time
            self.latency_list.append(latency)
            self.latency_file.write(f"{latency:.6f}\n")

            if len(self.latency_list) > 10000:
                self.latency_list.pop(0)
            """
            # Calculate steering angle from (x, y) output
            x = xy[0]
            y = (0.5 - xy[1]) / 2.0
            angle = np.arctan2(x, y)

            # Create Twist message and publish
            twist = Twist()
            twist.linear.x = self.vel
            twist.angular.z = angle
            self.twist_pub.publish(twist)

        except Exception as e:
            self.get_logger().error(f'Failed to process image: {e}')

        """    
        def report_latency_stats(self):
        if not self.latency_list:
            return
        try:
            min_l = min(self.latency_list)
            max_l = max(self.latency_list)
            mean_l = statistics.mean(self.latency_list)
            std_l = statistics.stdev(self.latency_list) if len(self.latency_list) > 1 else 0.0

            self.get_logger().info(
                f"Latencia inferencia (s) → Min: {min_l:.6f}, Max: {max_l:.6f}, "
                f"Mean: {mean_l:.6f}, Std: {std_l:.6f}"
            )
        except Exception as e:
            self.get_logger().error(f"Error en estadísticas de latencia: {e}")
        """
        """    
        def on_exit(self):
        if self.latency_list:
            min_l = min(self.latency_list)
            max_l = max(self.latency_list)
            mean_l = statistics.mean(self.latency_list)
            std_l = statistics.stdev(self.latency_list) if len(self.latency_list) > 1 else 0.0

            resumen = (
                "\nResumen final de latencias de inferencia (s):\n"
                f"Min:  {min_l:.6f}\n"
                f"Max:  {max_l:.6f}\n"
                f"Mean: {mean_l:.6f}\n"
                f"Std:  {std_l:.6f}\n"
            )

            print(resumen)

            if self.latency_file:
                self.latency_file.write(resumen)
                self.latency_file.close()
        else:
            if self.latency_file:
                self.latency_file.close()

        """
def main(args=None):
    rclpy.init(args=args)
    node = SteeringNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Steering node stopped manually.')
    finally:
        """node.on_exit()"""
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

