cmake_minimum_required(VERSION 3.0.2)
project(diff_robot_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
)

## The catkin_package macro generates cmake config files for your package
catkin_package(
   INCLUDE_DIRS include
)

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Add keyboar_teleop node
add_executable(keyboard_teleop 
  src/keyboard_teleop.cpp
  src/KeyInterpreter.cpp)
add_dependencies(keyboard_teleop ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(keyboard_teleop ${catkin_LIBRARIES})
