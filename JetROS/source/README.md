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

<table>
<tr>
<td width="40%">
<img src="fig/ROS2_System.png" alt="Image 1" style="width:100%;"/>
</td>
<td width="60%">
<p>
ROS 2 system diagram of the agent.
</p>
</td>
</tr>
</table>

---

### Neural Network

<table>
<tr>
<td width="40%">
<img src="path/to/image2.png" alt="Image 2" style="width:100%;"/>
</td>
<td width="60%">
<p>
Explanation or description for section 2. This can include instructions, descriptions of physical parts, software components, etc.
</p>
</td>
</tr>
</table>

---

### Action Prioritizer

<table>
<tr>
<td width="40%">
<img src="path/to/image3.png" alt="Image 3" style="width:100%;"/>
</td>
<td width="60%">
<p>
More details about this part of the project. Keep using this format to maintain a clear and organized README.
</p>
</td>
</tr>
</table>

---
### PID Controller

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
### Motor Controller

<table>
<tr>
<td width="40%">
<img src="path/to/image3.png" alt="Image 3" style="width:100%;"/>
</td>
<td width="60%">
<p>
More details about this part of the project. Keep using this format to maintain a clear and organized README.
</p>
</td>
</tr>
</table>

---
### Lidar Filter

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
### Odometry

<table>
<tr>
<td width="40%">
<img src="path/to/image3.png" alt="Image 3" style="width:100%;"/>
</td>
<td width="60%">
<p>
More details about this part of the project. Keep using this format to maintain a clear and organized README.
</p>
</td>
</tr>
</table>

---
### Encoder R and L

<table>
<tr>
More details about this part of the project. Keep using this format to maintain a clear and organized README.
<tr>
</table>

---


