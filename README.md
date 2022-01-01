# UAV Communication Tools
## Introduction
This toolkit is (will be) a simple set of inter-ROS-communication programs. It provides data & instruction exchanging between ROS systems by using network protocols.

By now, only the first version is presented.

## Usage
- Make sure you have correctly installed ROS and catkin tools.
- Create direction of workspace on your home direction, or you may create direction anywhere you like by changing the directions in the CMake list.
- Create direction `src` in the new direction.
- Copy the files into `src`.
- Run `catkin_make` on workspace direction.
- Source `src` in bash.
- Run program as all ROS programs.

## Version 1.0
Provides simple, dumb, best-effort communication service.

The 1.0 version contains 7 files:
- `CMakeLists.txt` which contains compile information.
- `ros_server_sub.cpp` -> `server_sub`
- `ros_server_pub.cpp` -> `server_pub`
- `ros_client_sub.cpp` -> `client_sub`
- `ros_client_pub.cpp` -> `client_pub`
- `test_sub.cpp` -> `test_sub`
- `test_pub.cpp` -> `test_pub`

When using, please first compile and run two server programs on your server machine, under program package `uav_communication`. Then start client programs on another machine.

To communicate, the main ROS progress should publish the data on topic `uav_message_out` and subscribe data on topic `uav_message_in`.

The last two test program will be a dumb simulator of ROS progress. `test_pub` publishes simple message frequently to topic `uav_message_out` while `test_pub` subscribes messages from topic `uav_message_in`.
