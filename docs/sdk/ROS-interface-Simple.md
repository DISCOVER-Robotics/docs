---
sidebar: auto
---

# ROS Interface

The ROS adapter layer for [`arm-control`](https://git.qiuzhi.tech/airbot-play/control/arm-control)

## 1. Install

### 1.1 Install latest `arm-control`

Install the deb packages from [releases of `arm-control`](https://git.qiuzhi.tech/airbot-play/control/arm-control/-/releases)

### 1.2 (Optional) Install depencencies of JoyStick

Run with root privileges:

```shell
apt install ros-noetic-joy ros-noetic-tf ros-noetic-kdl-parser
```

### 1.3 Create ROS workspace and install ROS interface

```shell
git clone --recursive https://git.qiuzhi.tech/airbot-play/control/sdk.git
cd sdk
ln -s /your_path/sdk/ros /path/to/your/workspace/src/
cd /path/to/your/workspace
catkin_make
```

Now you can control the arm by ROS and JoyStick.

**Note**: The default urdf does not contain end effector. If you wish to perform precise end control, a custom urdf file with `link6` as the target end is necessary.

## 2. Usage

### 2.1 ROS Topic Reference

#### ROS Topic of Publisher
- `/airbot_play/set_target_pose`: type ->  `geometry_msgs::Pose`, publish target end pose relative to base
- `/airbot_play/set_target_joint_q`: type ->  `sensor_msgs::JointState`, publish target position of joints
- `/airbot_play/set_target_joint_v`: type ->  `sensor_msgs::JointState`, publish target velocity of joints
- `/airbot_play/gripper/set_position`: type ->  `std_msgs::Float64`, publish target status of gripper, `0` means close, `1` means open

#### ROS Topic of Subscriber
- `/airbot_play/end_pose`: type ->  `geometry_msgs::Pose`, subscribe current end pose relative to base
- `/airbot_play/joint_states`: type ->  `sensor_msgs::JointState`, subscribe current position of joints
- `/airbot_play/gripper/position`: type ->  `std_msgs::Float64`, subcribe current status of gripper

#### ROS Topic Usage Eg.
- First, set up ros-interface environment
  ```shell
  cd /path/to/your/workspace/
  source devel/setup.bash
  roslaunch ros_interface airbot_arm.launch end_mode:=gripper [urdf:=</path/to/urdf/file>]
  ```
  ```Note```: ```end_mode``` types includes  ```gripper```  ```newtracher```  ```yinshi``` . Select parameters according to your corresponding hardware type.

- See files in folder `tests` corresponding to three control mode.
- c++
  ```shell
  rosrun ros_interface airbot_test
  ```
- python
  ```shell
  python ./tests/py_example.py
  ```
- cmd example
  ```shell
  python ./tests/cmd_example.py
  ```
- cmd 
  ```shell
  rostopic pub -1 /airbot_play/set_target_pose geometry_msgs/Pose "position:
  x: 0.55
  y: 0.10
  z: 0.35
  orientation:
  x: 0.0
  y: 0.707
  z: 0.0
  w: 0.707" 
  ```
  
  ```shell
  rostopic pub -1 /airbot_play/gripper/set_position std_msgs/Float64 "data: 1.0" 
  ```