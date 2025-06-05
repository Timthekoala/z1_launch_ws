cmake_minimum_required(VERSION 3.0.2)
project(robot_arm_controller)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)

## Declare a catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES robot_arm_controller
  CATKIN_DEPENDS roscpp geometry_msgs std_msgs
  DEPENDS EIGEN3
)

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
)

link_directories(${PROJECT_SOURCE_DIR}/lib)

## Declare a C++ executable
add_executable(robot_arm_controller_node src/z1_ros_wrapper.cpp)

catkin_install_python(PROGRAMS src/robot_arm_control.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Specify libraries to link a library or executable target against
target_link_libraries(robot_arm_controller_node
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${PROJECT_SOURCE_DIR}/lib/libZ1_SDK_x86_64.so
)



