cmake_minimum_required(VERSION 3.0.2)
project(manipulator_hw)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  cmake_modules 
  hardware_interface 
  controller_manager 
  control_msgs 
  std_msgs 
)

catkin_package(
 INCLUDE_DIRS 
    include  
 LIBRARIES 
    manipulator_hw
 CATKIN_DEPENDS 
    roscpp 
    hardware_interface
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
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/manipulator_hw.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

## Declare a C++ executable
add_executable(manipulator_hw_node
  src/manipulator_hw_node.cpp
)

set_target_properties(manipulator_hw_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Specify libraries to link a library or executable target against
target_link_libraries(manipulator_hw_node 
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
add_dependencies(manipulator_hw_node  ${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})





