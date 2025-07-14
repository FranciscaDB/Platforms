import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Bool
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
import threading
import time
import os
import signal
import csv
from datetime import datetime

# ROS 2 Node that handles subscription and publication of velocity and distance data
class VelocityPlotNode(Node):
    def __init__(self):
        super().__init__('velocity_plot_node')
        self.target1 = 0.0
        self.read1 = 0.0
        self.target2 = 0.0
        self.read2 = 0.0
        self.dist_read = 0.0

        # Subscriptions to velocity references and readings
        self.create_subscription(Twist, '/jetros1/vel_ref', self.target1_callback, 5)
        self.create_subscription(Twist, '/jetros1/vel_read', self.read1_callback, 5)
        self.create_subscription(Twist, '/jetros2/vel_ref', self.target2_callback, 5)
        self.create_subscription(Twist, '/jetros2/vel_read', self.read2_callback, 5)
        self.create_subscription(Float32, '/jetros2/dist_read', self.dist_callback, 5)

        # Publishers to control velocities and start signal
        self.vel1_pub = self.create_publisher(Float32, '/jetros1/vel', 2)
        self.vel2_pub = self.create_publisher(Float32, '/jetros2/vel', 2)
        self.start_pub = self.create_publisher(Bool, '/start', 2)

    # Callback methods to update internal state with incoming messages
    def target1_callback(self, msg): self.target1 = msg.linear.x
    def read1_callback(self, msg): self.read1 = msg.linear.x
    def target2_callback(self, msg): self.target2 = msg.linear.x
    def read2_callback(self, msg): self.read2 = msg.linear.x
    def dist_callback(self, msg): self.dist_read = msg.data * 100

    # Publishing methods
    def publish_vel1(self, vel): self.vel1_pub.publish(Float32(data=float(vel)))
    def publish_vel2(self, vel): self.vel2_pub.publish(Float32(data=float(vel)))
    def publish_start(self, state): self.start_pub.publish(Bool(data=state))

# Initialize ROS 2
rclpy.init()
node = VelocityPlotNode()
executor = SingleThreadedExecutor()
executor.add_node(node)

# CSV file setup for data logging
csv_path = f'velocity_data_{datetime.now().strftime("%Y%m%d_%H%M%S")}.csv'
csv_file = open(csv_path, mode='w', newline='')
csv_writer = csv.writer(csv_file)
csv_writer.writerow(['time', 'vel_ref1', 'vel_read1', 'vel_ref2', 'vel_read2', 'dist_read'])

# GUI window initialization
root = tk.Tk()
root.title("Control and Monitoring JETROS")
root.geometry("900x1000")

# Create Matplotlib figure and subplots
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(8, 8))
fig.subplots_adjust(hspace=0.7, bottom=0.1)
canvas = FigureCanvasTkAgg(fig, master=root)
canvas.get_tk_widget().pack()

# Setup axes for each subplot
for ax, title in zip([ax1, ax2], ['Velocity JETROS1', 'Velocity JETROS2']):
    ax.set_xlim(0, 60)
    ax.set_ylim(-5, 20)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Velocity (cm/s)")
    ax.set_title(title)
    ax.grid(True)

ax3.set_xlim(0, 60)
ax3.set_ylim(20, 40)
ax3.set_xlabel("Time (s)")
ax3.set_ylabel("Distance (cm)")
ax3.set_title("Distance JETROS2")
ax3.grid(True)

# Initialize plot lines
line_ref1, = ax1.plot([], [], label='ref', color='blue')
line_read1, = ax1.plot([], [], label='read', color='red')
ax1.legend()

line_ref2, = ax2.plot([], [], label='ref', color='blue')
line_read2, = ax2.plot([], [], label='read', color='red')
ax2.legend()

line_dist_ref, = ax3.plot([], [], label='dist_ref', color='green', linestyle='--')
#line_dist_tol_upper, = ax3.plot([], [], color='orange', linestyle='--')
#line_dist_tol_lower, = ax3.plot([], [], color='orange', linestyle='--')
line_dist_read, = ax3.plot([], [], label='dist_read', color='purple')
ax3.legend()

# Data containers for plotting
x_vals = []
ref1_vals = []
read1_vals = []
ref2_vals = []
read2_vals = []
dist_vals = []
dist_ref_vals = []
#dist_tol_upper_vals = []
#dist_tol_lower_vals = []

absolute_time_start = time.time()
first_start_time = None
started = False
after_id = None

# Periodically update plots and log data
def update_plot():
    global after_id, first_start_time, started
    now_abs = time.time()
    if not started:
        return
    now_rel = now_abs - first_start_time if first_start_time else 0
    now_csv = now_abs - absolute_time_start

    executor.spin_once(timeout_sec=0.01)

    # Append latest data to buffers
    x_vals.append(now_rel)
    ref1_vals.append(node.target1)
    read1_vals.append(node.read1)
    ref2_vals.append(node.target2)
    read2_vals.append(node.read2)
    dist_vals.append(node.dist_read)
    dist_ref_vals.append(30.0)

    # Update plot data
    line_ref1.set_data(x_vals, ref1_vals)
    line_read1.set_data(x_vals, read1_vals)
    line_ref2.set_data(x_vals, ref2_vals)
    line_read2.set_data(x_vals, read2_vals)
    line_dist_ref.set_data(x_vals, dist_ref_vals)
    line_dist_read.set_data(x_vals, dist_vals)

    # Adjust X axis to current time
    for ax in [ax1, ax2, ax3]:
        ax.set_xlim(0, now_rel)
    canvas.draw()

    # Write to CSV
    csv_writer.writerow([f"{now_csv:.2f}", node.target1, node.read1, node.target2, node.read2, node.dist_read])

    # Schedule next update
    after_id = root.after(100, update_plot)

# Helper to create labeled sliders
def make_slider(label_text, var, command):
    ttk.Label(root, text=label_text).pack()
    label = ttk.Label(root, text=f"{label_text}: 0")
    label.pack()
    slider = ttk.Scale(root, from_=0, to=20, variable=var, orient='horizontal', length=600)
    slider.pack(pady=5)
    def update_label(val):
        val = round(float(val))
        label.config(text=f"{label_text}: {val}")
    slider.config(command=update_label)
    def on_release(e):
        val = round(slider.get())
        slider.set(val)
        command(val)
    slider.bind("<ButtonRelease-1>", on_release)

# Create velocity sliders
vel1_var = tk.IntVar(value=0)
make_slider("JETROS1 Velocity", vel1_var, node.publish_vel1)

vel2_var = tk.IntVar(value=0)
make_slider("JETROS2 Velocity", vel2_var, node.publish_vel2)

# Control buttons for Start and Stop
frame = tk.Frame(root)
frame.pack(pady=10)

def start_pressed():
    global started, first_start_time
    if not started:
        started = True
        first_start_time = time.time()
        update_plot()
    node.publish_start(True)

ttk.Button(frame, text="Start", command=start_pressed).pack(side=tk.LEFT, padx=20)
ttk.Button(frame, text="Stop", command=lambda: node.publish_start(False)).pack(side=tk.LEFT, padx=20)

# Handle GUI window close event
def on_close():
    global after_id
    if after_id:
        root.after_cancel(after_id)
    csv_file.close()
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()
    root.destroy()
    os.kill(os.getpid(), signal.SIGTERM)

root.protocol("WM_DELETE_WINDOW", on_close)
root.mainloop()
