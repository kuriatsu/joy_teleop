#!/bin/bash

process=`pgrep record`
kill -2 $process

echo "rosbag stopped"
