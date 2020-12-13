#!/bin/bash

today=`date +'%Y_%m_%d'`
time=`date +'%H_%M_%S'`
dir=/media/kuriatsu/SamsungKURI/master_study_bag/$today

if [ -e $dir ]; then
	echo "file found"
else
	mkdir -p $dir
fi

echo "record_start"
rosbag record /managed_objects /wall_object /feedback_object /ff_target /joy /fake_control_cmd /carla/ego_vehicle/odometry /carla/ego_vehicle/vehicle_control_cmd -O $dir/$time
# rosbag record /shifted_info /closest_obstacle /detected_obstacles /managed_obstacles /next_waypoint_mark /swipe_erase_signal /twist_raw /ypspur_ros/cmd_vel /ypspur_ros/odom /points_raw /joy -O $dir/$time
