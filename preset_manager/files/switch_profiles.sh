#!/bin/sh

set -x
gazebo --verbose preset_example.world -o ode_quick_75iters &

for i in {1..3}; do
  sleep 7
  gz physics -o ode_world_300iters
  sleep 7
  gz physics -o ode_quick_75iters
done
set +x
