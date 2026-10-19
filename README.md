
# UR5e MoveIt-Gazebo Co-simulation: Aruco Marker Tracking with Hand-Eye Calibration
ROS Noetic simulation project for UR5e robotic arm, integrating MoveIt motion planning, Gazebo simulation, Aruco marker pose estimation and easy_handeye eye-in-hand calibration.

This repository is developed for robot learning and graduation design. The system mounts a monocular camera on the UR5e end-effector, detects Aruco marker, calibrates the extrinsic parameter between camera and tool0, and drives the manipulator to track the target marker in real time.

## Features
- UR5e Gazebo simulation environment based on official universal_robots package
- MoveIt motion planning & execution in simulation (`sim:=true`)
- Eye-in-hand monocular camera modeled in URDF/Xacro
- Aruco marker detection and TF pose publishing
- easy_handeye hand-eye calibration to get camera-tool0 transformation
- Real-time target tracking with MoveGroup C++ interface
- Python script for generating Aruco marker image

## Environment
- Ubuntu 20.04
- ROS Noetic
- Gazebo 11
- OpenCV 4.2.0
- Aruco module built in OpenCV

## Dependencies
Install ROS official packages:
```bash
sudo apt update
sudo apt install ros-noetic-universal-robots ros-noetic-moveit-simple-controller-manager
sudo apt install ros-noetic-aruco-ros ros-noetic-rqt-joint-trajectory-controller
```

Clone extra source packages into your catkin workspace:

```
cd ~/catkin_ws/src
git clone https://gitee.com/wang212_0/easy_handeye.git
git clone <this-repository-url>
cd ..
catkin_make
source devel/setup.bash
```

## Quick Start

### 1. Generate Aruco Marker

```
rosrun ur5e_gazebo make_marker.py
```

It outputs marker23.png under the script folder.

### 2. Launch Gazebo and MoveIt

Terminal 1:

```
roslaunch ur_gazebo ur5e_bringup.launch
```

Terminal 2:

```
roslaunch ur5_moveit_config moveit_planning_execution.launch sim:=true
```

Terminal 3 (RViz):

```
rviz
```

### 3. Hand-eye Calibration (Eye-in-hand)

Run aruco marker detection:

```
roslaunch aruco_ros single.launch \
markerId:=23 \
markerSize:=0.1 \
eye:="right" \
ref_frame:=/camera_link
```

Launch easy_handeye GUI, collect multiple robot poses, then save the calibration yaml file.
Copy the saved yaml into `ur5e_gazebo/config/` for later use.

> 
> Note: Without loading the calibrated transformation, the marker TF will be unstable and inaccurate.

### 4. Run Marker Tracking

Load hand-eye calibration yaml, start Aruco detection, then launch tracking node:

```
# C++ tracker
rosrun ur5e_gazebo ur5e_aruco_tracker
# Python detection node (alternative)
rosrun ur5e_gazebo aruco_single_py.py
```

The UR5e arm will plan trajectories and follow the Aruco marker automatically.

## Project Structure

```
ur5e_gazebo/
├── launch/                 # Launch files for simulation, tracking and handeye loading
├── urdf/                   # Modified UR5e xacro with mounted monocular camera
├── config/                 # Hand-eye calibration yaml result
├── scripts/                # Python scripts: marker generation, aruco detection
├── src/                    # C++ source code for MoveIt tracking node
├── CMakeLists.txt
└── package.xml
```
<img width="1381" height="1170" alt="2026-10-17-13-30-20-image" src="https://github.com/user-attachments/assets/3710067f-c94f-4341-8471-4e279cd3f3dd" />
<img width="1018" height="786" alt="2026-10-17-13-48-02-image" src="https://github.com/user-attachments/assets/677c8b17-d5d3-472b-8dc6-d9f7da5951b8" />
<img width="1646" height="1014" alt="2026-10-17-13-45-29-image" src="https://github.com/user-attachments/assets/1aa69815-b24d-4b8c-aa56-409c7afcb0da" />
<img width="1843" height="1160" alt="2026-10-17-16-38-59-image" src="https://github.com/user-attachments/assets/6a61cdcf-5da1-4a52-b38c-5ea8f00beb30" />
<img width="2462" height="1392" alt="2026-10-17-15-51-27-image" src="https://github.com/user-attachments/assets/da33c4e1-8002-4d9c-a0a7-f66bec241754" />


## Key Notes & Pitfalls

1. This is **eye-in-hand** setup: camera is attached to end-effector tool0.
2. Hand-eye calibration is required to get the transformation between `tool0` and `camera_link`.
3. `sim:=true` must be set to use simulated controller, do NOT connect to real UR robot.
4. Collect robot poses with different orientations and positions for better calibration accuracy.
