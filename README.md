# 🤖 UR5e–YOLO Vision Integration

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)
![Python](https://img.shields.io/badge/Python-Control-yellow)
![YOLOv5](https://img.shields.io/badge/YOLOv5-Object%20Detection-green)
![UR5e](https://img.shields.io/badge/Robot-UR5e-red)

A real-time vision-guided robotic system integrating:

- **UR5e robotic arm**
- **Intel RealSense camera**
- **YOLO-based object detection**
- **Python control node (ROS 2)**

The system detects objects and dynamically adjusts robot behavior based on detection confidence.

---

## Features

- Real-time RGB image streaming (RealSense)
- YOLO-based object detection
- Confidence-driven control logic
- ROS 2 → URScript robot control pipeline
- Modular ROS 2 node architecture

---

## System Architecture

```mermaid
graph LR
    A[RealSense Camera] -->|Image Topic| B[YOLO Detection Node]
    B -->|Detection Results| C[Confidence Bridge Node]
    C -->|/confidence| D[Python Control Node]
    D -->|URScript Commands| E[UR5e Robot]

    subgraph ROS2 Ecosystem
        B
        C
        D
    end
```

---



## Getting Started

### Prerequisites

- ROS 2 (Humble or compatible)
- Intel RealSense SDK
- YOLOv5 ROS package
- Python 3.10 with ROS 2 Python packages (`rclpy`, `std_msgs`)
- NumPy
- UR5e robot connected to network

---

## ▶️ Launch Instructions


### 1️⃣ Launch RealSense Camera

```bash
ros2 launch realsense2_camera rs_pointcloud_launch.py
```

**Optional (custom resolution):**
```bash
ros2 launch realsense2_camera rs_pointcloud_launch.py rgb_camera.color_profile:=1280x720x6
```

- **Topic:** `/camera/camera/color/image_raw`
- **Description:** Publishes RGB image stream for object detection

---

### 2️⃣ Launch UR5e Driver

```bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=158.125.190.88
```

- **Network:**
  - University network: `158.125.190.88`
  - Internal robot IP: `10.0.0.100`

- **Description:** Enables ROS 2 communication with the UR5e robot

---

### 3️⃣ Launch YOLO Detection Node

```bash
ros2 run yolov5_ros yolov5_ros --ros-args --remap raw:=/camera/camera/color/image_raw
```

**Alternative:**
```bash
ros2 launch yolov5_ros2 yolov5_ros2
```

- **Description:**
  - Subscribes to camera images
  - Performs object detection
  - Outputs detection results with confidence scores

---

### 4️⃣ Launch Confidence Bridge Node

```bash
ros2 run confid_subpub confid_subpub
```

- **Publishes:** `/confidence`
- **Description:**
  - Extracts confidence from detection results
  - Publishes processed value for control logic

---

### 5️⃣ Launch Python Control Node

```bash
source /opt/ros/humble/setup.bash
python3.10 ur5e_ros2_lookAt_control.py
```

If `python3.10` is not on your PATH, use the system Python directly:

```bash
source /opt/ros/humble/setup.bash
/usr/bin/python3.10 ur5e_ros2_lookAt_control.py
```

- **Publishes:** `/urscript_interface/script_command`
- **Description:**
  - Computes a predefined sequence of viewpoint poses
  - Orients the UR5e TCP so its tool Z axis points at a fixed target point
  - Sends URScript commands to UR5e

---

## 🔄 System Pipeline

```
Camera → Detection → Confidence → Control → Robot
```

The experimental setting is given in the following figure:
![ROS2 System Diagram](Figures/expt_setting.png)


---

## Debugging & Verification

Check active topics:
```bash
ros2 topic list
```

Monitor confidence output:
```bash
ros2 topic echo /confidence
```

Common issues:

- ❌ No detection → Check image topic remapping  
- ❌ Robot not moving → Verify `/urscript_interface/script_command`  
- ❌ No camera feed → Confirm RealSense node is running  

---

<!-- ## 📁 Project Structure (Suggested)

```
.
├── yolov5_ros/
├── confid_subpub/
├── ur5e_ros2_lookAt_control.py
├── launch/
├── README.md
```

--- -->


## Author

- Yalei Yu (y.yu2@lboro.ac.uk) – Loughborough University
- Yuchuan Jin (y.jin2@lboro.ac.uk) – Loughborough University

---

## License

This project is licensed under the MIT License 
