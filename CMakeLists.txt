cmake_minimum_required(VERSION 2.8.3)
project(joy_teleop)

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
joy
dynamic_reconfigure
g29_force_feedback
carla_msgs
)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )


## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
cfg/joy_teleop.cfg
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


add_executable(joy_teleop_node src/joy_teleop.cpp)

add_dependencies(joy_teleop_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} joy_teleop_gencfg)
target_link_libraries(joy_teleop_node ${catkin_LIBRARIES})
