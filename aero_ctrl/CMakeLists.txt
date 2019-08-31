cmake_minimum_required(VERSION 2.8.3)
project(aero_ctrl)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  control_toolbox
  angles
  message_filters
  message_generation
  std_msgs
  bebop_msgs
  geometry_msgs
)
find_package(Eigen3 REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Debug.msg
  DebugFil.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES aero_ctrl
  CATKIN_DEPENDS roscpp rospy control_toolbox std_msgs angles bebop_msgs geometry_msgs message_runtime
#  DEPENDS system_lib
  DEPENDS EIGEN3
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

## Declare a C++ executable, add cmake target dependencies of the library, and
## specify libraries to link a library or executable to target against
add_executable(beb_pos_ctrl_node src/beb_pos_ctrl.cpp src/beb_pos_ctrl_node.cpp)
add_dependencies(beb_pos_ctrl_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(beb_pos_ctrl_node
  ${catkin_LIBRARIES}
)

add_executable(filter_ext_node src/filter_ext.cpp src/filter_ext_node.cpp)
add_dependencies(filter_ext_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(filter_ext_node
  ${catkin_LIBRARIES}
)

add_executable(beb_pos_smc_node src/beb_pos_smc.cpp src/beb_pos_smc_node.cpp)
add_dependencies(beb_pos_smc_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(beb_pos_smc_node
  ${catkin_LIBRARIES}
)

add_executable(beb_pos_pid_node src/beb_pos_pid.cpp src/beb_pos_pid_node.cpp)
add_dependencies(beb_pos_pid_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(beb_pos_pid_node
  ${catkin_LIBRARIES}
)
