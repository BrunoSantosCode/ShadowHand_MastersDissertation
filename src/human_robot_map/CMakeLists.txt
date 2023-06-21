cmake_minimum_required(VERSION 3.0.2)
project(human_robot_map)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  bio_ik
  message_generation
  geometry_msgs
  moveit_core
  moveit_ros_move_group
  moveit_ros_planning
  moveit_ros_planning_interface
  roscpp
  Eigen3
)

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
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
add_message_files(
  FILES
  HandKeypoints.msg
)

## Generate services in the 'srv' folder
#add_service_files(
#  FILES
#  fk.srv
#  checkSelfCollision.srv
#)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
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
  CATKIN_DEPENDS 
  moveit_core 
  moveit_ros_planning  
  bio_ik moveit_ros_planning 
  roscpp 
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  $ENV{EIGEN3_INCLUDE_DIR}
)

#include_directories(include)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(hand_kp_pub src/hand_kp_pub.cpp)
add_executable(bio_ik_v0 src/bio_ik_v0.cpp)
add_executable(bio_ik_v1 src/bio_ik_v1.cpp)
add_executable(bio_ik_v2 src/bio_ik_v2.cpp)
add_executable(bio_ik_v3 src/bio_ik_v3.cpp)
add_executable(plot_keypoints src/plot_keypoints.cpp)
add_executable(bio_ik_v4 src/bio_ik_v4.cpp)
add_executable(bio_ik_v5 src/bio_ik_v5.cpp)
add_executable(bio_ik_v6 src/bio_ik_v6.cpp)
add_executable(bio_ik_v7 src/bio_ik_v7.cpp)
add_executable(bio_ik_v8 src/bio_ik_v8.cpp)
add_executable(bio_ik_v10 src/bio_ik_v10.cpp)
add_executable(bio_ik_v11 src/bio_ik_v11.cpp)
add_executable(bio_ik_v12 src/bio_ik_v12.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(hand_kp_pub ${catkin_EXPORTED_TARGETS})
add_dependencies(bio_ik_v0 ${catkin_EXPORTED_TARGETS})
add_dependencies(bio_ik_v1 ${catkin_EXPORTED_TARGETS})
add_dependencies(bio_ik_v2 ${catkin_EXPORTED_TARGETS})
add_dependencies(bio_ik_v3 ${catkin_EXPORTED_TARGETS})
add_dependencies(plot_keypoints ${catkin_EXPORTED_TARGETS})
add_dependencies(bio_ik_v4 ${catkin_EXPORTED_TARGETS})
add_dependencies(bio_ik_v5 ${catkin_EXPORTED_TARGETS})
add_dependencies(bio_ik_v6 ${catkin_EXPORTED_TARGETS})
add_dependencies(bio_ik_v7 ${catkin_EXPORTED_TARGETS})
add_dependencies(bio_ik_v8 ${catkin_EXPORTED_TARGETS})
add_dependencies(bio_ik_v10 ${catkin_EXPORTED_TARGETS})
add_dependencies(bio_ik_v12 ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(hand_kp_pub 
  ${catkin_LIBRARIES}
)
target_link_libraries(bio_ik_v0 
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)
target_link_libraries(bio_ik_v1 
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)
target_link_libraries(bio_ik_v2 
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)
target_link_libraries(bio_ik_v3 
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)
target_link_libraries(plot_keypoints 
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)
target_link_libraries(bio_ik_v4
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)
target_link_libraries(bio_ik_v5
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)
target_link_libraries(bio_ik_v6
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)
target_link_libraries(bio_ik_v7
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)
target_link_libraries(bio_ik_v8
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)
target_link_libraries(bio_ik_v10
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)
target_link_libraries(bio_ik_v11
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)
target_link_libraries(bio_ik_v12
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)