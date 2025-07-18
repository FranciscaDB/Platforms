# ROS2 packages, GUI and more
This section contains all information related to the ROS 2 packages, beginning with the installation of necessary third-party dependencies, and then providing details about the jetros package.

### Diagram of the complete system:
<p align="center">
  <img src="fig/ROS2_System.png" alt="ROS 2 system diagram of the agent." style="width:70%;"/>
</p>

# Third-party packages and necessary changes

## Camera Node Package    
The package for the camera node can be found in the following GitHub repository:
```bash
https://github.com/ros-drivers/usb_cam
```
This package must be built from source, as we need access to its source files in order to make modifications.
The required modification is described below:
Navigate to the file located at:
```bash
ros2_ws/src/usb_cam/src/usb_cam_node.cpp
```
Find the following line:
```bash
const char BASE_TOPIC_NAME[] = "image_raw";
```
And change it to:
```bash
const char BASE_TOPIC_NAME[] = "/jetrosX/image_raw";
```
Where X is the ID of your robot. This ID must match the one used in the launch file of the jetros package.
After making the change, you must recompile the package for the modification to take effect.

## LiDAR Node Package
The package for the LiDAR node can be found in the following GitHub repository:
```bash
https://github.com/Slamtec/rplidar_ros/tree/ros2
```
Follow the instructions provided in the repository to compile and install the package.

Once installed, navigate to the file located at:
```bash
ros2_ws/src/rplidar_ros/src/rplidar_node.cpp
```
Find the following lines:
```cpp
this->declare_parameter<std::string>("topic_name", std::string("scan"));
this->get_parameter_or<std::string>("topic_name", topic_name, "scan");
```
And change them to:
```cpp
this->declare_parameter<std::string>("topic_name", std::string("/jetrosX/scan"));
this->get_parameter_or<std::string>("topic_name", topic_name, "/jetrosX/scan");
```
Where `X` is the ID of your robot. This ID must match the one used in the launch file of the `jetros` package.

After making the changes, you must recompile the package for them to take effect.

# jetros package
---

### Launch

This launch file is meant to initialize and run the full sensor and control stack for a robot in the jetros system. It also allows configuring the robot's ID and adjusting certain parameters of the LiDAR filter, such as the distance range and the angular sector in which the data is filtered. These settings are explained in more detail in the node section.

---

### Neural Network Node

This node, called steering_node, is responsible for processing images from a camera and generating coordinates corresponding to the center of the path. It uses a pre-trained neural network optimized with TensorRT to infer the steering direction from the images and publishes the result as a Twist message. Note that the model path must be updated to match the correct location of the .pth file on your system.

---

### Action Prioritizer Node

More details about this part of the project. Keep using this format to maintain a clear and organized README.

---
### PID Controller Node

<table>
<tr>
<td width="40%">
<img src="fig/PID.png" alt="Image 3" style="width:100%;"/>
</td>
<td width="60%">
<p>
More details about this part of the project. Keep using this format to maintain a clear and organized README.
</p>
</td>
</tr>
</table>

---
### Motor Controller Node

More details about this part of the project. Keep using this format to maintain a clear and organized README.

---
### Lidar Filter Noder

<table>
<tr>
<td width="40%">
<img src="fig/Lidar_filter.png" alt="Image 3" style="width:100%;"/>
</td>
<td width="60%">
<p>
More details about this part of the project. Keep using this format to maintain a clear and organized README.
</p>
</td>
</tr>
</table>

---
### Odometry Node

More details about this part of the project. Keep using this format to maintain a clear and organized README.

---
### Encoder R and L Nodes

More details about this part of the project. Keep using this format to maintain a clear and organized README.

---


