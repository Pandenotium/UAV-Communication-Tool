# toplevel CMakeLists.txt for a catkin workspace
# catkin/cmake/toplevel.cmake

cmake_minimum_required(VERSION 3.0.2)

project(uav_communication)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs genmsg)

## Declare ROS messages and services
#add_message_files(FILES Num.msg)
#add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_workspace()

## a socket ros app
include_directories(include ${catkin_INCLUDE_DIRS}
			/usr/include/eigen3
			/opt/ros/melodic/include
)
add_executable(server_pub $ENV{HOME}/workspace/src/ros_server_pub.cpp)
target_link_libraries(server_pub ${catkin_LIBRARIES})
add_executable(server_sub $ENV{HOME}/workspace/src/ros_server_sub.cpp)
target_link_libraries(server_sub ${catkin_LIBRARIES})
add_executable(client_pub $ENV{HOME}/workspace/src/ros_client_pub.cpp)
target_link_libraries(client_pub ${catkin_LIBRARIES})
add_executable(client_sub $ENV{HOME}/workspace/src/ros_client_sub.cpp)
target_link_libraries(client_sub ${catkin_LIBRARIES})
add_executable(test_pub $ENV{HOME}/workspace/src/test_pub.cpp)
target_link_libraries(test_pub ${catkin_LIBRARIES})
add_executable(test_sub $ENV{HOME}/workspace/src/test_sub.cpp)
target_link_libraries(test_sub ${catkin_LIBRARIES})
