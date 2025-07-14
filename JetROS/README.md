# JETROS

JETROS is a modular and scalable robotic platform designed for real-time experimentation with autonomous platooning strategies. Built on the NVIDIA Jetson Orin Nano and ROS 2, it integrates a 2D LiDAR, wheel encoders, and an RGB camera to enable perception through neural networks, computer vision, and sensor fusion. Based on a differential drive robot model, the system supports inter-vehicle communication, remote monitoring, and reconfiguration, making it well-suited for studying key challenges such as collision avoidance and spacing control on curved paths. Its efficient and extensible architecture enables the development and testing of control and perception algorithms in multi-agent autonomous systems.

<p align="center">
  <img src="jetros.png" alt="image" width="400"/>
</p>

## Recommended Workflow
To successfully follow this project, we suggest progressing in the following order:

🔧 env_setup/ – Environment setup (dependency installation, ROS 2 setup, Jetson configuration, etc.)

🛠️ physical_build/ – Physical build of the robot (3D printing files, assembly details, wiring, etc.)

📁 source/ – Copy the corresponding files from the source folder to the ROS 2 workspace.

🧠 training/ - Copy the training files and follow the instructions in the README.

🔳 Launch the GUI from a robot or from a computer configured with ROS 2 and that has the same ROS ID. 

And with this, you'll be able to use the JETROS platform.


