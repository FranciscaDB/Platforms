import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Twist
import time
import statistics
from rclpy.clock import Clock
import atexit

class ActionPioritizer(Node):

    def __init__(self):
        super().__init__('action_prioritizer_node')

        # Get ID parameter
        self.declare_parameter('ID', 2)
        self.ID = self.get_parameter('ID').get_parameter_value().integer_value
        
        # Internal state variables
        self.start = False
        self.object_detected = False
        self.object_distance = float('inf')  
        self.target_twist = Twist()
        self.jetros_twist = Twist()

        # PID control variables for distance tracking
        self.d_ref = 30.0
        self.deadband = 1
        self.error_ant_d = 0.0
        self.integral_d = 0.0
        self.Kp_d = 1.5455
        self.Ki_d = 0.040574
        self.Kd_d = 0.0
        self.sat_d = 15.0
        self.clock = Clock()
        self.last_time = self.clock.now()

       # Subscribers for start command, detection, distance, and velocity targets
        self.create_subscription(Bool, '/start', self.start_callback, 5)
        self.create_subscription(Bool, f'/jetros{self.ID}/obj_det', self.object_detected_callback, 1)
        self.create_subscription(Float32, f'/jetros{self.ID}/dist_read', self.object_distance_callback, 1)  
        self.create_subscription(Twist, f'/jetros{self.ID}/target', self.target_callback, 1)
        self.create_subscription(Twist, f'/jetros{self.ID-1}/vel_ref', self.jetros_callback, 1) ## THIS IS NOT ALWAYS ID-1

        # Publisher for velocity reference output
        self.velocity_pub = self.create_publisher(Twist, f'/jetros{self.ID}/vel_ref', 1)

        # Periodic timer for control loop
        self.timer = self.create_timer(0.1, self.timer_callback)

        """        
        # PID latency logging
        self.latency_list = []
        self.latency_file = open('latencias_pid_dist.txt', 'w')
        self.latency_file.write("latency_seconds\n")
        atexit.register(self.on_exit)

         # Periodic latency statistics logging
        self.create_timer(10.0, self.report_latency_stats)
        """
    def start_callback(self, msg):
        self.start = msg.data

    def object_detected_callback(self, msg):
        self.object_detected = msg.data

    def object_distance_callback(self, msg):
        self.object_distance = msg.data * 100 # Convert from meters to cm

    def target_callback(self, msg):
        self.target_twist = msg

    def jetros_callback(self, msg):
        self.jetros_twist = msg
        
    # PID controller implementation for distance regulation
    def calculoPIDd(self, y, ref, error_ant, error_integral, kp, ki, kd, limite, dt):
        error = y - ref
        error_integral += error * dt
        integral_term = ki * error_integral
        if integral_term > limite:
            error_integral = limite / ki
        elif integral_term < -limite:
            error_integral = -limite / ki
        u = kp * error + kd * (error - error_ant) / dt + ki * error_integral
        error_ant = error
        gamma = 0.1
        upper_limit = limite * (1 + gamma)
        lower_limit = -limite * (1 + gamma)
        u = max(min(u, upper_limit), lower_limit)
        return u, error_ant, error_integral

    # Main control loop callback
    def timer_callback(self):
        current_time = self.clock.now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        output_twist = Twist()

        if self.start:
            if self.object_detected:
                # Object too close, stop the robot
                if self.object_distance < 23.0:
                    output_twist.linear.x = 0.0
                    output_twist.angular.z = 0.0
                else:
                    error_dist = self.object_distance - self.d_ref
                    if abs(error_dist) > self.deadband:
                        # Run PID controller if error exceeds deadband
                        #start_time = time.time()
                        vel_output, self.error_ant_d, self.integral_d = self.calculoPIDd(
                            self.object_distance, self.d_ref,
                            self.error_ant_d, self.integral_d,
                            self.Kp_d, self.Ki_d, self.Kd_d,
                            self.sat_d, dt)
                        """                        
                        latency = time.time() - start_time
                        self.latency_list.append(latency)
                        self.latency_file.write(f"{latency:.6f}\n")
                        """
                        output_twist.linear.x = vel_output
                    else:
                        # Use leader robot's reference velocity
                        output_twist.linear.x = self.jetros_twist.linear.x
                    output_twist.angular.z = self.target_twist.angular.z
            else:
                 # No object detected, use default target velocity
                output_twist.linear.x = self.target_twist.linear.x
                output_twist.angular.z = self.target_twist.angular.z
        else:
            # Robot is stopped
            output_twist.linear.x = 0.0
            output_twist.angular.z = 0.0
            
        # Publish calculated velocity command
        self.velocity_pub.publish(output_twist)

        """        
        # Limit list size to avoid memory issues
        if len(self.latency_list) > 10000:
            self.latency_list.pop(0)
        """
    """ 
    # Periodically print latency statistics for PID controller
    def report_latency_stats(self):
        if not self.latency_list:
            return
        try:
            min_l = min(self.latency_list)
            max_l = max(self.latency_list)
            mean_l = statistics.mean(self.latency_list)
            std_l = statistics.stdev(self.latency_list) if len(self.latency_list) > 1 else 0.0

            self.get_logger().info(
                f"Latencia PID distancia (s) → Min: {min_l:.6f}, Max: {max_l:.6f}, "
                f"Mean: {mean_l:.6f}, Std: {std_l:.6f}"
            )
        except Exception as e:
            self.get_logger().error(f"Error calculando estadísticas: {e}")
    """
    """ 
    # Cleanup and final statistics report at shutdown
    def on_exit(self):
        if self.latency_file:
            try:
                if self.latency_list:
                    min_l = min(self.latency_list)
                    max_l = max(self.latency_list)
                    mean_l = statistics.mean(self.latency_list)
                    std_l = statistics.stdev(self.latency_list) if len(self.latency_list) > 1 else 0.0

                    resumen = (
                        "\nResumen final de latencias PID distancia (s):\n"
                        f"Min:  {min_l:.6f}\n"
                        f"Max:  {max_l:.6f}\n"
                        f"Mean: {mean_l:.6f}\n"
                        f"Std:  {std_l:.6f}\n"
                    )
                    print(resumen)
                    self.latency_file.write(resumen)
                self.latency_file.close()
            except Exception as e:
                self.get_logger().error(f"Error al cerrar archivo de latencias: {e}")
        else:
            if self.latency_file:
                self.latency_file.close()
    """
def main(args=None):
    rclpy.init(args=args)
    node = ActionPioritizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        """node.on_exit()"""
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

