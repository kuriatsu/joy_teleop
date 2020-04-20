#!/bin/bash
today=`date +'%Y_%m_%d'`
time=`date +'%H_%M_%S'`
dir=/media/kuriatsu/SamsungKURI/master_study_bag/$today

if [ ! -f $dir/$time.bag.active ]; then
    /home/kuriatsu/Program/Ros/master_study_ws/src/teleop_study/src/qt
fi
