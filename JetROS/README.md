# JETROS

JETROS is a modular and scalable robotic platform designed for real-time experimentation with autonomous platooning strategies. Built on the NVIDIA Jetson Orin Nano and ROS 2, it integrates a 2D LiDAR, wheel encoders, and an RGB camera to enable perception through neural networks, computer vision, and sensor fusion. Based on a differential drive robot model, the system supports inter-vehicle communication, remote monitoring, and reconfiguration, making it well-suited for studying key challenges such as collision avoidance and spacing control on curved paths. Its efficient and extensible architecture enables the development and testing of control and perception algorithms in multi-agent autonomous systems.

<p align="center">
  <img src="jetros.png" alt="image" width="400"/>
</p>

## Requirements

The detailed hardware specifications and assembly instructions are provided in the physical_build/ section.

Please note that JETROS has only been tested on the NVIDIA Jetson Orin Nano platform; compatibility with other Jetson platforms has not been verified.

## How to run the system

In the RTEMSOFT lab setup, both JETROS agents connect to a Wi-Fi router named `JETBOT_TP_LINK`.  
The password for this network is `JetbotPSW`. This connection is essential for enabling SSH access.

Alternatively, a display, keyboard, and mouse can be connected directly to the Jetson Orin Nano, allowing it to be used as a standalone computer. 

**Warning:** Since the standoffs are made of plastic, they are not very robust. Therefore, it is strongly advised to handle the JETROS units with great care, always holding them by the base and not by the upper floor.



To connect via SSH (after waiting several seconds for the Jetson Orin Nano to fully boot), use the following commands:

```bash
ssh -X fran@192.168.0.102
ssh -X nelson@192.168.0.103
```

Tip: You can use Ctrl + R to search through the terminal history on the JETROS units and quickly find previously used commands.

If you wish to browse directories, extract files, or modify code, it is recommended to run the following command from the HOME directory:
```bash
jupyter lab --ip=0.0.0.0 --port=8888 --no-browser --NotebookApp.token='' --NotebookApp.password=''
```

Tip: It is recommended to use a computer running Ubuntu 22.04 with ROS 2 Foxy in order to launch the GUI from that machine. To access topics published by the JETROS agents, make sure the ROS Domain ID is set to 1 on both systems. If this is not possible, the GUI can also be launched directly from one of the robots.

To run the ROS 2 system, simply execute on each robot the following command:
```bash
ros2 launch jetros launch_jetros.py
```

And from either the Linux computer with ROS 2 Foxy installed, or from the selected robot, execute the following command (if executed from a robot, the script will be located in a folder named Monitoring within the HOME directory):
```bash
python3 gui.py
```

Finally, before pressing the START button on the GUI, it is necessary to press the button on the power bank, located at the back right side of the JETROS.
Once the button is pressed, you will have approximately 30 seconds to hit START before the power bank automatically shuts off again.

After using the JETROS, it is highly recommended to disconnect the USB cable from the PowerBank to avoid leakage during its sleeping mode.

It is strongly recommended to restart the launch process on each JETROS before every test, and only then run the GUI. Not restarting the launch or the GUI after the robots have been inactive for a while may lead to malfunctioning.

## Recommended Workflow
To successfully follow this project, we suggest progressing in the following order:

🔧 env_setup/ – Environment setup (dependency installation, ROS 2 setup, Jetson configuration, etc.)

🛠️ physical_build/ – Physical build of the robot (3D printing files, assembly details, wiring, etc.)

📁 source/ – Copy the corresponding files from the source folder to the ROS 2 workspace.

🧠 training/ - Copy the training files and follow the instructions in the README.

🔳 Launch the GUI from a robot or from a computer configured with ROS 2 and that has the same ROS ID. 

And with this, you'll be able to use the JETROS platform.
