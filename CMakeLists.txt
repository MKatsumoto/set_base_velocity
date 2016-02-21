cmake_minimum_required(VERSION 2.8.3)
project(set_base_velocity)

find_package(catkin REQUIRED COMPONENTS
#  dynamixel_msgs
  gecko_msgs
  geometry_msgs
  laser_geometry
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  set_state
  std_msgs
  tf
)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   dynamixel_msgs#   gecko_msgs#   geometry_msgs#   nav_msgs#   sensor_msgs#   std_msgs
# )

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES set_base_velocity
  CATKIN_DEPENDS
  #dynamixel_msgs
  gecko_msgs geometry_msgs laser_geometry nav_msgs roscpp rospy
  set_state sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(SetBaseVelocity
  src/SetBaseVelocity.cpp
)

# add_dependencies(set_base_velocity ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(set_base_velocity_node
  src/set_base_velocity_node.cpp
)

add_dependencies(set_base_velocity_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  set_state_generate_messages_cpp
  gecko_msgs_generate_messages_cpp
)


## Specify libraries to link a library or executable target against
target_link_libraries(set_base_velocity_node
  ${catkin_LIBRARIES}
  SetBaseVelocity
)
