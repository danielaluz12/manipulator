Cmake_minimum_required(VERSION 2.8.3)
project(tracik_planning)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  geometry_msgs
  visualization_msgs
  interactive_markers
  sensor_msgs
  #baxter_core_msgs
  visualization_msgs
  trac_ik_lib
  eigen_conversions
  kdl_conversions
  visualization_msgs
)

catkin_package(
 INCLUDE_DIRS 
 include
  # LIBRARIES
  CATKIN_DEPENDS
    trac_ik_lib
    kdl_conversions
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
   ${catkin_INCLUDE_DIRS}
   ${eigen_INCLUDE_DIRS}
)



## build end_effector_marker_control
add_executable(ik_tests_main
    src/ik_tests.cpp
    src/ik_tests_main.cpp
)


add_dependencies(ik_tests_main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(ik_tests_main  
  ${catkin_LIBRARIES}
  )

## build trac_ik_test.cpp
# add_executable(end_effector_trac_ik_solver src/end_effector_trac_ik_solver.cpp)
# target_link_libraries(end_effector_trac_ik_solver ${catkin_LIBRARIES})