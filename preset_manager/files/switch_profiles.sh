#!/bin/sh

set -x
gazebo --verbose preset_example.world -o ode_200iters &

echo start with 200 iterations
sleep 5
gz physics -o ode_70iters
echo switch to 70 iterations
echo simulation will go unstable
sleep 5
echo switch back to 200 iterations
gz physics -o ode_200iters
echo reset world
gz world -r

echo open plot window to show link pitch and yaw angles
sleep 10
echo
for i in $(seq 5)
do
  gz physics -o ode_500iters
  sleep 5
  gz physics -o ode_200iters
  sleep 5
done
set +x
