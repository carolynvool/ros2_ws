# TurtleBot3 Mapping Process

## Overview
This document explains how to set up and run the mapping process using a TurtleBot3 robot in a 3D simulation environment with ROS2 Humble on Ubuntu 22.04. The process includes simulation configuration, SLAM implementation, and autonomous navigation for mapping.

## Prerequisites
- ROS2 Humble  
- Gazebo  
- Cartographer  
- Python  
- Ubuntu 22.04  

## Running the Simulation (All inside a Docker container)

### 1. Launch Gazebo Simulation
`ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py`

### 2. (Optional) Manual Control  
Run this in a separate terminal:  
`ros2 run turtlebot3_teleop teleop_keyboard`

### 3. Visualize in RViz2  
`ros2 launch turtlebot3_bringup rviz2.launch.py`

## SLAM Mapping Process

### 1. Launch Simulation World  
`ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py`

### 2. Run the SLAM Node  
`ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True`

### 3. Move the Robot  
Either manually:  
`ros2 run turtlebot3_teleop teleop_keyboard`  
or using the autonomous mapping node (see below).

### 4. Save the Map  
After mapping is complete:  
`ros2 run nav2_map_server map_saver_cli -f ~/map`

## Autonomous Mapping Node

### 1. Include `mapping.py`  
Ensure the `mapping.py` file is inside your `my_robot_controller` package.

### 2. Add Node to `setup.py`  
Register the mapping node inside `setup.py`.

### 3. Build and Run the Node  
`colcon build --packages-select my_robot_controller`  
`source install/setup.bash`  
`ros2 run my_robot_controller mapping`

## Launch File for Complete Mapping Process

### 1. Create Launch File `start_mapping.launch.py`  
This file should include all necessary nodes.

### 2. Run Complete Mapping Process  
`ros2 launch my_robot_controller start_mapping.launch.py`
