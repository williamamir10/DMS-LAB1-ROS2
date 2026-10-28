Design of Mechatronic Systems — Lab 1

ROS 2, Turtlesim, Actions and Services

Project Overview

This project implements a ROS 2 control system using Turtlesim. It demonstrates custom interfaces, action servers, services, runtime speed configuration, and launch files.

Workspace Structure

dms_ws/
└── src/
    ├── lab1_interfaces/
    │   ├── msg/
    │   │   └── Target.msg
    │   ├── srv/
    │   │   └── SetSpeedLimits.srv
    │   └── action/
    │       ├── GoToPose.action
    │       └── DrawSquare.action
    ├── lab1_control/
    │   ├── lab1_control/
    │   │   ├── turtle_controller.py
    │   │   └── square_server.py
    │   ├── launch/
    │   │   └── lab1.launch.py
    │   ├── package.xml
    │   └── setup.py
    └── README.md

Implemented Features

* Target message: Defines the target position and orientation using x, y, and yaw.
* GoToPose action: Moves the turtle to a requested pose and provides feedback and a result.
* DrawSquare action: Draws a square by sending four sequential GoToPose goals.
* Reset service: Clears the drawing and resets the turtle.
* Speed limits service: Updates the maximum linear and angular speeds.
* Launch file: Starts Turtlesim, the turtle controller, and the square action server.

Requirements

* Ubuntu with ROS 2 Jazzy
* Turtlesim
* colcon build tools

Build Instructions

Open a terminal and run:

source /opt/ros/jazzy/setup.bash
cd ~/dms_ws
colcon build
source install/setup.bash

Run the Project

Start all project nodes using the launch file:

source /opt/ros/jazzy/setup.bash
source ~/dms_ws/install/setup.bash
ros2 launch lab1_control lab1.launch.py

Test the GoToPose Action

In another terminal:

source /opt/ros/jazzy/setup.bash
source ~/dms_ws/install/setup.bash
ros2 action send_goal /go_to_pose lab1_interfaces/action/GoToPose "{target: {x: 6.544, y: 5.544, yaw: 0.0}}" --feedback

Test the DrawSquare Action

ros2 action send_goal /draw_square lab1_interfaces/action/DrawSquare "{side_length: 2.0}" --feedback

Test the Reset Service

ros2 service call /reset_turtle std_srvs/srv/Empty "{}"

Test the Speed Limits Service

ros2 service call /set_speed_limits lab1_interfaces/srv/SetSpeedLimits "{max_linear_speed: 1.0, max_angular_speed: 2.0}"

Notes

* Build the workspace before running the nodes.
* Source the ROS 2 and workspace setup files in each new terminal.
* The build/, install/, and log/ directories should not be committed to the repository.
* Verify all action and service results before submission.
