cmake_minimum_required(VERSION 3.0.2)
project(manipulator_vision)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  # serial
  tf
  tf2
  tf2_ros
  tf2_msgs
  tf2_geometry_msgs
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
)



# Generate Service message
add_service_files(
   FILES
   RequestEEPos.srv
)

generate_messages(
	DEPENDENCIES
	std_msgs
)



catkin_package(
  #INCLUDE_DIRS include
#  LIBRARIES subscriber_python
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

# add_executable(${PROJECT_NAME}_node_2 src/manipulator_vision_node_2.cpp)

# ## Rename C++ executable without prefix
# ## The above recommended prefix causes long target names, the following renames the
# ## target back to the shorter version for ease of user use
# ## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

# ## Add cmake target dependencies of the executable
# ## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# ## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )


add_executable(manipulator_vision_node_2
#include custom_hi/manip_hw_interface.h
src/manipulator_vision_node_2.cpp
)

add_dependencies(manipulator_vision_node_2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(manipulator_vision_node_2
${catkin_LIBRARIES}
)



# add_executable(manipulator_vision_trans
# #include custom_hi/manip_hw_interface.h
# src/manipulator_vision_trans.cpp
# )

# add_dependencies(manipulator_vision_trans ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(manipulator_vision_trans
# ${catkin_LIBRARIES}
# )


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  scripts/parseserial3D.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


