# ROS2 Humble Arm Control Package
This repository holds the scripts used for controlling a 6DoF arm over RF comms using UDP channels for streaming the joint_states of the hardware state interfaces and updateing a robot model in an RVIZ2 instance.

![image](https://github.com/user-attachments/assets/4ab9ae3f-6060-4654-9bf1-0ea5b7f2a950)

## Control
For controlling the arm, the [joint_state_publisher_gui](http://wiki.ros.org/joint_state_publisher_gui) package is used. This streams the desired joint states and these are send over RF communications to the arm, where a PID script published effort commands to ros2 controllers.

## Control System Flow Diagram
![image](https://github.com/user-attachments/assets/3647f595-8db1-405d-a179-6c3e21c59efd)

## Hardware Set Up
![image](https://github.com/user-attachments/assets/de9e33ef-bf9d-4f77-89f7-fcf621aa22ed)


## Launch
1. First clone the repository inside of your src folders in a working ros2 worksapce.
2. Build and soruce the worksapce
```
colcon build && source install/setup.bash
```
3. In the robotic arm launch the rover launch script
```
ros2 launch arm_control rover_arm_hardware.luanch.py
```
4. At the base station launch the following launch script
```
ros2 launch arm_control base_station.launch.py
```
