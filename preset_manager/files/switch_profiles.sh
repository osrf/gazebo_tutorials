gazebo preset_world.world -o ode_world_70iters &

for i in {1..10}; do
  sleep 5
  gz physics -o ode_quick_30iters
  sleep 5
  gz physics -o ode_world_70iters
done
