#!/bin/sh

set -x
gazebo --verbose preset_example_sdf1_6.world -o ode_70iters &

for i in {1..5}; do
  sleep 5
  gz physics -o ode_200iters
  sleep 5
  gz physics -o ode_70iters
done
set +x
