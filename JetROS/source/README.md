# ROS2 packages, GUI and more
This section contains all information related to the ROS 2 packages, beginning with the installation of necessary third-party dependencies, and then providing details about the jetros package.

<img src="fig/ROS2_System.png" alt="ROS 2 system diagram of the agent." style="width:70%;"/>

# Third-party packages and necessary changes

## Camer Node Package
The package for the camera node can be found in the following GitHub repository:
https://github.com/ros-drivers/usb_cam
This package must be built from source, as we need access to its source files in order to make modifications.
The required modification is described below:
Navigate to the file located at:
´´´bash
ros2_ws/src/usb_cam/src/usb_cam_node.cpp
´´´
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

### Section 2 Title

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

### Section 3 Title

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

