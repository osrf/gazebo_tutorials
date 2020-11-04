#! /bin/bash

mkdir revolute_joint_test || true
wget https://raw.githubusercontent.com/osrf/gazebo/diagnostics_scpeters/test/worlds/revolute_joint_test.world
for i in 0 1 2 3 4 5 6
do
gzserver --verbose -o unthrottled$i revolute_joint_test.world \
  & sleep 2 && gz topic -e /gazebo/default/diagnostics | grep real_time_factor | awk -F: '{print $2}' > revolute_joint_test/unthrottled$i.csv \
  & sleep 30 ; killall -9 gazebo gzclient gzserver gz
done

for i in 0 1 2 3 4 5 6
do
gzserver --verbose -o split_unthrottled$i revolute_joint_test.world \
  & sleep 2 && gz topic -e /gazebo/default/diagnostics | grep real_time_factor | awk -F: '{print $2}' > revolute_joint_test/split_unthrottled$i.csv \
  & sleep 30 ; killall -9 gazebo gzclient gzserver gz
done
