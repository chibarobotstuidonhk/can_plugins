cmake_minimum_required(VERSION 2.8.3)
project(can_plugins)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
  nodelet
  pluginlib
  roscpp
)

add_message_files(
  DIRECTORY msg
  FILES
  Frame.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES can_plugins
  CATKIN_DEPENDS nodelet pluginlib roscpp std_msgs message_runtime
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/slcan_bridge.cpp
  src/can_handler.cpp
  src/ping.cpp
  src/md201x_node.cpp
)

add_dependencies(${PROJECT_NAME} ${TARGET_NAME} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(solenoid_driver_node src/solenoid_driver_node.cpp)
target_link_libraries(solenoid_driver_node ${catkin_LIBRARIES})