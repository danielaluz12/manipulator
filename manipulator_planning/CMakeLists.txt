Cmake_minimum_required(VERSION 2.8.3)
project(manipulator_planning)

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

# Build the ManipulatorIK library
add_library(${PROJECT_NAME}
    src/ManipulatorIK.cpp
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

## Add executable node
add_executable(manipulator_planning_node
    src/manipulator_planning_node.cpp
)
add_dependencies(manipulator_planning_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(manipulator_planning_node
    ${PROJECT_NAME}
)

add_executable(ik_tests_class
    test/ik_tests_class.cpp
)
add_dependencies(ik_tests_class ${catkin_EXPORTED_TARGETS})
target_link_libraries(ik_tests_class
    ${PROJECT_NAME}
)

# Test the inverse kinematic for arbitrary end-effector positions
add_executable(ik_tests
    test/ik_tests.cpp
)
add_dependencies(ik_tests ${catkin_EXPORTED_TARGETS})
target_link_libraries(ik_tests
    ${PROJECT_NAME}
)


