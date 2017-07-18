cmake_minimum_required(VERSION 2.8.3)
project(apc_nozel)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  joint_state_publisher
  moveit_core
  moveit_msgs
  moveit_planners_ompl
  moveit_ros_move_group
  moveit_ros_planning_interface
  moveit_ros_visualization
  robot_state_publisher
  ur_description
  xacro
  message_generation
  std_msgs
  rospy
  roscpp
  geometry_msgs
  sensor_msgs
  actionlib
  actionlib_msgs
  control_msgs
  visualization_msgs
 pcl_ros
    catkin


)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   actuator_frame2.srv
   get_points2.srv
   computer_vision.srv
   gripper_suction_controller.srv
   placement.srv
   CheckClearProtectiveStop.srv
write_stow_data.srv
stowToteContents.srv
motor_input.srv
 )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
#)


## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
   sensor_msgs
   actionlib_msgs
   control_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES apc_nozel
  CATKIN_DEPENDS roscpp std_msgs
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
 include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(apc_nozel
#   src/${PROJECT_NAME}/apc_nozel.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(apc_nozel ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(apc_nozel_node src/apc_nozel_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(apc_nozel_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(apc_nozel_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS apc_nozel apc_nozel_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_apc_nozel.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)




#add_executable(actuator2 src/actuator2.cpp)
add_executable(actuator_point_server2 src/actuator_point_server2.cpp)
add_executable(actuator_point_server2_for_ur5 src/actuator_point_server2_for_ur5.cpp)
#add_executable(demo_actuator2 src/demo_actuator2.cpp)
add_executable(publish_joint2 src/publish_joint2.cpp)
add_executable(get_points_server2 src/get_points_server2.cpp)
#add_executable(kinnect_frame2 src/kinnect_frame2.cpp)
#add_executable(test_actuator2 src/test_actuator2.cpp)
add_executable(joint_state_feedback src/joint_state_feedback.cpp)
#add_executable(force src/force.cpp)
#add_executable(samarat_sir src/samarat_sir.cpp)
#add_executable(demo_actuator2_moveit src/demo_actuator2_moveit.cpp)
add_executable(demo_actuator2_moveit_force src/demo_actuator2_moveit_force.cpp)
add_executable(automate_v2 src/automate_v2.cpp)
add_executable(automate_v3 src/automate_v3.cpp)
add_executable(automate_v1 src/automate_v1.cpp)
add_executable(automate_v4 src/automate_v4.cpp)
add_executable(automate_v3_for_ur5 src/automate_v3_for_ur5.cpp)
add_executable(automate_v1_for_ur5 src/automate_v1_for_ur5.cpp)
add_executable(test_json src/test_json.cpp)


add_executable(ur_communication src/ur_communication.cpp)





#add_dependencies(demo_actuator2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_dependencies(actuator2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})




target_link_libraries(actuator_point_server2_for_ur5 ${catkin_LIBRARIES})
target_link_libraries(test_json ${catkin_LIBRARIES})

target_link_libraries(automate_v3_for_ur5 ${catkin_LIBRARIES})
target_link_libraries(automate_v1_for_ur5 ${catkin_LIBRARIES})

target_link_libraries(automate_v1 ${catkin_LIBRARIES})
target_link_libraries(automate_v2 ${catkin_LIBRARIES})
target_link_libraries(automate_v3 ${catkin_LIBRARIES})
target_link_libraries(automate_v4 ${catkin_LIBRARIES})

target_link_libraries(ur_communication ${catkin_LIBRARIES})
target_link_libraries(demo_actuator2_moveit_force ${catkin_LIBRARIES})
#target_link_libraries(demo_actuator2_moveit ${catkin_LIBRARIES})
#target_link_libraries(samarat_sir ${catkin_LIBRARIES})
#target_link_libraries(force ${catkin_LIBRARIES})
target_link_libraries(joint_state_feedback ${catkin_LIBRARIES})
#target_link_libraries(test_actuator2 ${catkin_LIBRARIES})
#target_link_libraries(kinnect_frame2 ${catkin_LIBRARIES})
target_link_libraries(get_points_server2 ${catkin_LIBRARIES})
target_link_libraries(publish_joint2 ${catkin_LIBRARIES})
#target_link_libraries(demo_actuator2 ${catkin_LIBRARIES})
target_link_libraries(actuator_point_server2 ${catkin_LIBRARIES})
#target_link_libraries(actuator2 ${catkin_LIBRARIES})
