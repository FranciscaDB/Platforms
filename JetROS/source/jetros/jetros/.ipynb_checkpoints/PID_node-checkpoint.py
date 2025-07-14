import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist
import time
import statistics
import atexit

class PIDControllerNode(Node):
    def __init__(self):
        super().__init__('pid_node')
        
        # Get robot ID parameter
        self.declare_parameter('ID', 2)
        self.ID = self.get_parameter('ID').get_parameter_value().integer_value

        # Subscriptions to velocity references and readings
        self.subscription_velocity_ref = self.create_subscription(
            Twist, f'/jetros{self.ID}/vel_ref', self.velocity_ref_callback, 1)

        self.subscription_velocity_read = self.create_subscription(
            Twist, f'/jetros{self.ID}/vel_read', self.velocity_read_callback, 1)

         # Publisher to command motor outputs
        self.publisher_motors = self.create_publisher(Twist, f'/jetros{self.ID}/motors', 2)

        # Velocity and angular references and inputs
        self.vel_ref = 0.0
        self.theta_ref = 0.0
        self.Input_vel = 0.0
        self.Input_theta = 0.0

        # PID parameters for velocity
        self.error_ant_vel = 0.0
        self.integral_vel = 0.0
        self.Kp_vel = 8.4653
        self.Ki_vel = 25.8202
        self.Kd_vel = 0.0
        self.sat_vel = 200.0

        # PID parameters for angular velocity
        self.error_ant_theta = 0.0
        self.integral_theta = 0.0
        self.Kp_theta = 40.7204
        self.Ki_theta = 3.7421
        self.Kd_theta = 4.325
        self.sat_theta = 255.0

        # Control loop timer
        self.timer_period = 0.1  # 100 ms
        self.timer = self.create_timer(self.timer_period, self.control_loop)
        self.last_time = time.time()
        """
        # Latency tracking for both PID controllers
        self.latency_vel_list = []
        self.latency_theta_list = []
        self.latency_file = open('latencias_pid_vel_theta_separado.txt', 'w')
        self.latency_file.write("latency_vel_seconds,latency_theta_seconds\n")
        atexit.register(self.on_exit)

        # Periodic report of latency statistics
            self.create_timer(10.0, self.report_latency_stats)
        """

    def velocity_ref_callback(self, msg):
        # Update velocity and angular references
        self.vel_ref = msg.linear.x
        self.Input_theta = msg.angular.z

    def velocity_read_callback(self, msg):
        # Update current measured velocity
        self.Input_vel = msg.linear.x

    def control_loop(self):
        # Compute time delta
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        # Measure PID latency for velocity
        #start_vel = time.time()
        
        Output_vel, self.error_ant_vel, self.integral_vel = self.calculoPID(
            self.Input_vel, self.vel_ref, self.error_ant_vel, self.integral_vel,
            self.Kp_vel, self.Ki_vel, self.Kd_vel, self.sat_vel, "CONTROL_ON", 0.0, "DIRECTO", dt)
        #latency_vel = time.time() - start_vel

        # Measure PID latency for angular velocity
        #start_theta = time.time()
        Output_theta, self.error_ant_theta, self.integral_theta = self.calculoPID(
            self.Input_theta, self.theta_ref, self.error_ant_theta, self.integral_theta,
            self.Kp_theta, self.Ki_theta, self.Kd_theta, self.sat_theta, "CONTROL_ON", 0.0, "DIRECTO", dt)
        #latency_theta = time.time() - start_theta
        
        # Publish velocity commands
        msg = Twist()
        msg.linear.x = Output_vel
        msg.angular.z = Output_theta
        self.publisher_motors.publish(msg)

        """
        # Log latency values
        self.latency_vel_list.append(latency_vel)
        self.latency_theta_list.append(latency_theta)
        self.latency_file.write(f"{latency_vel:.6f},{latency_theta:.6f}\n")

        # Limit size of latency history
        if len(self.latency_vel_list) > 10000:
            self.latency_vel_list.pop(0)
        if len(self.latency_theta_list) > 10000:
            self.latency_theta_list.pop(0)
        """
    def calculoPID(self, y, ref, error_ant, error_integral, kp, ki, kd, limite, MODO, out_manual, direccion, dt):
        # PID control logic
        if MODO == "CONTROL_OFF":
            return out_manual, error_ant, error_integral
        
        error = ref - y if direccion == "DIRECTO" else y - ref
        error_integral += error * dt
        integral_term = ki * error_integral

        # Anti-windup
        if integral_term > limite:
            error_integral = limite / ki
        elif integral_term < -limite:
            error_integral = -limite / ki

        u = kp * error + kd * (error - error_ant) / dt + ki * error_integral
        error_ant = error
        
        # Output saturation
        u = max(min(u, limite), -limite)
        return u, error_ant, error_integral
    """
    def report_latency_stats(self):
        # Print latency statistics to ROS log
        if not self.latency_vel_list or not self.latency_theta_list:
            return
        try:
            min_vel = min(self.latency_vel_list)
            max_vel = max(self.latency_vel_list)
            mean_vel = statistics.mean(self.latency_vel_list)
            std_vel = statistics.stdev(self.latency_vel_list) if len(self.latency_vel_list) > 1 else 0.0

            min_theta = min(self.latency_theta_list)
            max_theta = max(self.latency_theta_list)
            mean_theta = statistics.mean(self.latency_theta_list)
            std_theta = statistics.stdev(self.latency_theta_list) if len(self.latency_theta_list) > 1 else 0.0

            self.get_logger().info(
                f"Latencia PID Velocidad (s) → Min: {min_vel:.6f}, Max: {max_vel:.6f}, Mean: {mean_vel:.6f}, Std: {std_vel:.6f}"
            )
            self.get_logger().info(
                f"Latencia PID Ángulo (s) → Min: {min_theta:.6f}, Max: {max_theta:.6f}, Mean: {mean_theta:.6f}, Std: {std_theta:.6f}"
            )
        except Exception as e:
            self.get_logger().error(f"Error calculando estadísticas: {e}")
    """
    """
    def on_exit(self):
        # Final summary written to file on shutdown
        if self.latency_file:
           try:
                if self.latency_vel_list and self.latency_theta_list:
                    min_vel = min(self.latency_vel_list)
                    max_vel = max(self.latency_vel_list)
                    mean_vel = statistics.mean(self.latency_vel_list)
                    std_vel = statistics.stdev(self.latency_vel_list) if len(self.latency_vel_list) > 1 else 0.0

                    min_theta = min(self.latency_theta_list)
                    max_theta = max(self.latency_theta_list)
                    mean_theta = statistics.mean(self.latency_theta_list)
                    std_theta = statistics.stdev(self.latency_theta_list) if len(self.latency_theta_list) > 1 else 0.0

                    resumen = (
                        "\nResumen final de latencias PID Velocidad (s):\n"
                        f"Min:  {min_vel:.6f}\n"
                        f"Max:  {max_vel:.6f}\n"
                        f"Mean: {mean_vel:.6f}\n"
                        f"Std:  {std_vel:.6f}\n\n"
                        "Resumen final de latencias PID Ángulo (s):\n"
                        f"Min:  {min_theta:.6f}\n"
                        f"Max:  {max_theta:.6f}\n"
                        f"Mean: {mean_theta:.6f}\n"
                        f"Std:  {std_theta:.6f}\n"
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
    node = PIDControllerNode()
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
