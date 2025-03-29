# TurtleBot3 Mapping Process
## Overview
This document explains how to set up and run the mapping process using a TurtleBot3 robot in a 3D simulation environment with ROS2 Humble on Ubuntu 22.04. The process includes simulation configuration, SLAM implementation and autonomous navigation for mapping. 
### Prerequisites
ROS2 Humble
Gazebo
Cartographer
Python
Ubuntu 22.04
## Running the Simulation (All inside a Docker container)
### 1. Launch Gazebo Simulation
`ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py`
### (2.) For manual control, run in a separate terminal
`ros2 run turtlebot3_teleop teleop_keyboard`
### 3. Visualise in RViz2
`ros2 launch turtlebot3_bringup rviz2.launch.py`
## SLAM Mapping Process
### 1. Launch Simulation World
`ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py`
### 2. Run the SLAM node
`ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True`
### 3. Control the robot
either with manual control 
`ros2 run turtlebot3_teleop teleop_keyboard`
or use the autonomous mapping node (shown below)
### 4. Save the map
after mapping is complete
`ros2 run nav2_map_server map_saver_cli -f ~/map`
## Autonomous Mapping Node
### 1. Include mapping.py file with provided code from [1] Mapping.pdf
### 2. Add node to setup.py
### 3. Build and run node again 
`ros2 run my_robot_controller mapping`
## Launch File for Complete Mapping Process
### 1. Create Launch file `start_mapping.launch.py`
### 2. Run complete mapping process
`ros2 launch my_robot_controller start_mapping.launch.py`
