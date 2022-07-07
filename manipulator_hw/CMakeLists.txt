cmake_minimum_required(VERSION 3.0.2)
project(manipulator_hw)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation 
  cmake_modules 
  hardware_interface 
  controller_manager 
  control_msgs 
  urdf 
  std_msgs 
  my_roscpp_library
)

catkin_package(
 INCLUDE_DIRS 
 include
#  LIBRARIES normal_hi
 CATKIN_DEPENDS 
    roscpp 
    hardware_interface
    std_msgs
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
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/normal_hi.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide



# add_library(normal_hw
#     src/normal_hw.cpp
# )



add_executable(normal_hw_main 
  src/normal_hw.cpp
  src/normal_hw_main.cpp
)
## Specify libraries to link a library or executable target against
target_link_libraries(normal_hw_main
  ${catkin_LIBRARIES}
)
add_dependencies(normal_hw_main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})





