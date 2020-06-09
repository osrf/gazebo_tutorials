#! /bin/bash

mkdir pr2 || true
wget https://raw.githubusercontent.com/osrf/gazebo/diagnostics_scpeters/worlds/pr2.world
for i in 0 1 2 3 4
do
gzserver --verbose -o unthrottled$i pr2.world \
  & sleep 2 && gz topic -e /gazebo/default/diagnostics | grep real_time_factor | awk -F: '{print $2}' > pr2/unthrottled$i.csv \
  & sleep 30 ; killall -9 gazebo gzclient gzserver gz
done

for i in 0 1 2 3 4
do
gzserver --verbose -o split_unthrottled$i pr2.world \
  & sleep 2 && gz topic -e /gazebo/default/diagnostics | grep real_time_factor | awk -F: '{print $2}' > pr2/split_unthrottled$i.csv \
  & sleep 30 ; killall -9 gazebo gzclient gzserver gz
done
