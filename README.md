
# RB-1 Robotnik Robot Simulation and Control in ROS and Gazebo

This repository contains a RB-1 Robotnik robot simulation using ROS (Robot Operating System) and Gazebo. The project encompasses creating a detailed URDF model of the robot, simulating it in Gazebo, and introducing a custom ROS node for easy control of the robot's rotation.

## Project Overview

The project is aimed at designing a simulated RB-1 Robotnik robot from scratch, providing mobile functionality with actuators and sensors, and including an out-of-the-box feature for easy rotation control suitable for users with minimal robotics programming experience.

### Getting Started

To begin with the project, clone this repository inside your `catkin_ws/src` directory:

```bash
cd ~/catkin_ws/src
git clone https://github.com/MiguelSolisSegura/my_rb1_robot.git
```

Ensure you have ROS and Gazebo installed on your system. This project was developed with ROS Noetic and Gazebo 11, but it should be compatible with other versions supporting ROS and Gazebo.

### Simulation and Control Features

- **URDF Creation**: Development of the URDF model for the RB-1 robot, detailing all required links, joints, actuators, and sensors.
- **Gazebo Simulation**: Integration of the robot model into a Gazebo world for simulation purposes.
- **Enhanced ROS Control**: Implementation of a custom ROS service for controlling the robot's rotation through simple commands.

### Testing the Project

Make sure to build the project using:

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

#### Displaying the Robot in RViz

To visualize the robot in RViz, use the following command:

```bash
roslaunch my_rb1_description display.launch
```

This command launches RViz where you can see the robot model and control the wheel joints through the `joint_state_publisher_gui`.

#### Simulating the Robot in Gazebo

To spawn the robot in the Gazebo simulation, execute:

```bash
roslaunch my_rb1_gazebo my_rb1_robot_warehouse.launch
```

This will launch a Gazebo simulation of a warehouse with the robot spawned inside.

#### Controlling Robot Rotation

To control the robot's rotation, first ensure the `rotate_service` server is running:

```bash
roslaunch my_rb1_ros rotate_service.launch
```

Then, to rotate the robot, use the `rosservice call` command with the desired degrees:

```bash
rosservice call /rotate_robot "degrees: 90"
rosservice call /rotate_robot "degrees: -90"
```

### Dependencies

- ROS (Robot Operating System)
- Gazebo
