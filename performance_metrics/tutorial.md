# Performance metrics

Gazebo 9 and Gazebo 11 publish a message called `/gazebo/performance_metrics` that allows to check the performance
of each sensor in the world.

# Learn how to read the measurements

We are going to compare the results between physics (contact) and rendering (camera) based sensors and the differences
when the `lockstep` flag is active.

## Camera

For the rendering-based sensor we are going to use the [camera_strict_rate.world](https://github.com/osrf/gazebo/blob/gazebo9/test/worlds/camera_strict_rate.world) world. It contains two cameras: a high-resolution (1280x720) high-frame-rate (500 fps) and a a low-resolution (320x240) low-frame-rate (30 fps) camera.

### Run the world without lockstep

With the `lockstep` flag deactivated there is no guarantee when the callback is called. This means the `sim_sensor_update_rate` could not correspond with the real value defined in the world. Inspect the `sim_sensor_update_rate` field, It is likely unable to reach the specified 500 fps. The other low-resolution camera is able to reach the specified value. Take note of the `real time factor` it should be close to 1.0.

```bash
real_time_factor: 0.99798848700000009
sensor {
  sensor_name: "default::camera_model::link::camera_sensor"
  real_sensor_update_rate: 235.6538640635398
  sim_sensor_update_rate: 250
  fps: 241.61639404296875
}
sensor {
  sensor_name: "default::camera_model::link::camera_sensor_regular"
  real_sensor_update_rate: 31.1463905584825
  sim_sensor_update_rate: 30.3030303030303
  fps: 29.975315093994141
}
```

### Run the world with lockstep

With the `lockstep` flag activated the `sim_sensor_update_rate` must correspond with the real value
defined in the world. In this case we expect a `sim_sensor_update_rate` equals to 500 in the high resolution camera
and 30 in the low resolution. Then the `fps` value means that Gazebo is able to generate 337 frames in a *real second*.

The real time factor is likely less than 1.0. The exact number depends on your computing power. This shows that
the sensor's update rate is strictly followed, and physics has slowed down in order to accommodate for the high update rate.

```bash
real_time_factor: 0.67489652
sensor {
  sensor_name: "default::camera_model::link::camera_sensor"
  real_sensor_update_rate: 348.7102949740385
  sim_sensor_update_rate: 500
  fps: 337.46530151367188
}
sensor {
  sensor_name: "default::camera_model::link::camera_sensor_regular"
  real_sensor_update_rate: 21.522750989820555
  sim_sensor_update_rate: 30.3030303030303
  fps: 20.173301696777344
}
```

### Real time update rate

We can modify the Real time update rate in the Physics Engine. As you may see the `sim_sensor_update_rate` in both cameras
are the expected value 500 and 30. In this case the Real time update rate is equal to **10** which make the simulation slower. The fps
are lower, it's able to generate almost 6 frames for the high speed camera and 2 for the regular camera each *real second*.

#### Run the world with lockstep

```bash
real_time_factor: 0.67666977300000009
sensor {
  sensor_name: "default::camera_model::link::camera_sensor"
  real_sensor_update_rate: 5.0383346478096511
  sim_sensor_update_rate: 500
  fps: 5.61544132232666
}
sensor {
  sensor_name: "default::camera_model::link::camera_sensor_regular"
  real_sensor_update_rate: 0.29411702534733497
  sim_sensor_update_rate: 29.411764705882351
  fps: 2.0637693405151367
}
```

#### Run the world without lockstep

Again in this case, where we run the world without `lockstep`, there is no guarantee when the callback is called,
and the `sim_sensor_update_rate` value is highly impacted, it should be close to 500 but in this case it's close to 0.

```bash
real_time_factor: 0.425736361
sensor {
  sensor_name: "default::camera_model::link::camera_sensor"
  real_sensor_update_rate: 96.478915160204693
  sim_sensor_update_rate: 0.14425851125216388
  fps: 80.7220458984375
}
sensor {
  sensor_name: "default::camera_model::link::camera_sensor_regular"
  real_sensor_update_rate: 169.40730989154036
  sim_sensor_update_rate: 0.14452955629426217
  fps: 15.246246337890625
}
```

#### max step size

The maximum time step size that can be taken by a variable time-step solver (such as simbody) during simulation.
For physics engines with fixed-step solvers (like ODE), this is simply the time step size. The default value
in Gazebo is 0.001 seconds.

When you choose the `update rate` of a sensor you need to take in account if you have enough precision
defined in `max step size`. For example:

  - 1/250 = 0.004 it's fine using the default value.
  - 1/400 = 0.0025. You will need to increase the `max step size` to 0.0001, this will make the simulation slower.

In the following traces we included a IMU sensor at 400Hz. You can see that the `sim_sensor_update_rate` does not correspond with the defined value
because the `max step size` does not have enough precision.

```bash
sensor {
  sensor_name: "default::camera_model::link::pendulum_imu_sensor"
  real_sensor_update_rate: 181.87525758083353
  sim_sensor_update_rate: 200
}
...
sensor {
  sensor_name: "default::camera_model::link::pendulum_imu_sensor"
  real_sensor_update_rate: 252.94164813236739
  sim_sensor_update_rate: 333.33333333333331
}
```

If we modify the `max step size` to 0.0001, then you will see the right value.

```bash
sensor {
  sensor_name: "default::camera_model::link::pendulum_imu_sensor"
  real_sensor_update_rate: 39.899370595408726
  sim_sensor_update_rate: 400
}
```
