cmake_minimum_required(VERSION 2.8.3)
project(arc_controller)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)


add_service_files(
  FILES
  camera_robot_calibration.srv
  points.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES arc_controller
  CATKIN_DEPENDS roscpp std_msgs
  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(arc_controller_node src/arc_controller.cpp)

target_link_libraries(arc_controller_node
  ${catkin_LIBRARIES}
)

