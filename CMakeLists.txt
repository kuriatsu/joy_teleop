cmake_minimum_required(VERSION 2.8.3)
project(teleop_carla)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
roscpp
rospy
std_msgs
geometry_msgs
nav_msgs
joy
dynamic_reconfigure
# carla_msgs
)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )


## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
cfg/teleop.cfg
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES swipe_obstacles
#  CATKIN_DEPENDS geometry_msgs roscpp std_msgs swipe_obstacles
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
${catkin_INCLUDE_DIRS}
)

add_executable(teleop_carla_ff_node src/teleop_carla_ff.cpp)
# add_dependencies(teleop_carla_ff_node teleop_gencfg)
target_link_libraries(teleop_carla_ff_node ${catkin_LIBRARIES})
