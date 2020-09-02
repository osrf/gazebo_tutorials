# Performance metrics

Gazebo 9 and Gazebo 11 publish a message called `/gazebo/performance_metrics` that allows to check the performance
of each sensor in the world.

# Learn how to read the measurements

We are going to compare the results between physics (contact) and rendering (camera) based sensors and the differences
when the `lockstep` flag is active.

## Camera

For the rendering-based sensor we are going to use the [camera_strict_rate.world](https://github.com/osrf/gazebo/blob/gazebo9/test/worlds/camera_strict_rate.world) world. It contains two cameras: a high-resolution (1280x720) high-frame-rate (500 fps) and a a low-resolution (320x240) low-frame-rate (30 fps) camera.

### Run the world without lockstep

With the `lockstep` flag deactivated there is no guarantee that the camera sensor update happens in the same iteration as the physics simulation. This means the `sim_sensor_update_rate` could not correspond with the real value defined in the world. Inspect the `sim_sensor_update_rate` field, It is likely unable to reach the specified 500 fps. The other low-resolution camera is able to reach the specified value. Take note of the `real time factor` it should be close to 1.0.

```bash
real_time_factor: 0.99798848700000009
sensor {
  sensor_name: "default::camera_model::link::camera_sensor"
  real_sensor_update_rate: 191.64276561974779
  sim_sensor_update_rate: 166.66666666666666
  fps: 213.16120910644531
}
sensor {
  sensor_name: "default::camera_model::link::camera_sensor_regular"
  real_sensor_update_rate: 31.1463905584825
  sim_sensor_update_rate: 30.3030303030303
  fps: 29.975315093994141
}
...
real_time_factor: 0.9913419670000001
sensor {
  sensor_name: "default::camera_model::link::camera_sensor"
  real_sensor_update_rate: 224.15729745202637
  sim_sensor_update_rate: 200
  fps: 213.16120910644531
}
sensor {
  sensor_name: "default::camera_model::link::camera_sensor_regular"
  real_sensor_update_rate: 31.294063018669757
  sim_sensor_update_rate: 31.25
  fps: 29.895561218261719
}
```

### Run the world with lockstep

With the `lockstep` flag activated the `sim_sensor_update_rate` must correspond with the real value
defined in the world. In this case we expect a `sim_sensor_update_rate` equals to 500 in the high resolution camera
and 30 in the low resolution. Then the `fps` value means that Gazebo is able to generate 337 frames in a *real second*.

The real time factor is likely less than 1.0. The exact number depends on your computing power. This shows that
the sensor's update rate is strictly followed, and physics has slowed down in order to accommodate for the high update rate.

```bash
real_time_factor: 0.5083145
sensor {
  sensor_name: "default::camera_model::link::camera_sensor"
  real_sensor_update_rate: 254.39207924822054
  sim_sensor_update_rate: 500
  fps: 279.38592529296875
}
sensor {
  sensor_name: "default::camera_model::link::camera_sensor_regular"
  real_sensor_update_rate: 12.626295945654704
  sim_sensor_update_rate: 30.3030303030303
  fps: 16.778518676757812
}
```

## Contact sensor

For the physics-based sensors we are going to use the [contact_strict_rate.world](https://github.com/osrf/gazebo/blob/gazebo9/test/worlds/contact_strict_rate.world) world. It contains a contact sensor (1000 Hz).

In this case both world look pretty similar, even with this high update rate. Because the update loop
is small in this case. This may vary with a more complex object

### Run the world without lockstep

```bash
real_time_factor: 0.988433204
sensor {
  sensor_name: "world_1::model_1::link_1::sensor_contact"
  real_sensor_update_rate: 964.34805249910789
  sim_sensor_update_rate: 1000
}
```

### Run the world with lockstep

```
real_time_factor: 1.003396497
sensor {
  sensor_name: "world_1::model_1::link_1::sensor_contact"
  real_sensor_update_rate: 1006.3885545442469
  sim_sensor_update_rate: 1000
}
```

## Real time update rate

We can modify the Real time update rate in the Physics Engine. As you may see the `sim_sensor_update_rate` in both cameras
are the expected value 500 and 30. In this case the Real time update rate is equal to **10** which make the simulation slower. The fps
are lower, it's able to generate almost 6 frames for the high speed camera and 2 for the regular camera each *real second*.

### Run the world with lockstep

```bash
real_time_factor: 0.969638674
sensor {
  sensor_name: "world_1::model_1::link_1::sensor_contact"
  real_sensor_update_rate: 972.45520628206054
  sim_sensor_update_rate: 1000
}

real_time_factor: 1.008738703
sensor {
  sensor_name: "world_1::model_1::link_1::sensor_contact"
  real_sensor_update_rate: 1011.0170528246299
  sim_sensor_update_rate: 1000
}
```

### Run the world without lockstep

Again in this case, where we run the world without `lockstep`, there is no guarantee that the camera sensor update happens
in the same iteration as the physics simulation, and the `sim_sensor_update_rate` value is highly impacted, it should be
close to 500 but in this case it's close to 0.

```bash
real_time_factor: 1.061299604
sensor {
  sensor_name: "world_1::model_1::link_1::sensor_contact"
  real_sensor_update_rate: 1065.2213743060083
  sim_sensor_update_rate: 1000
}

real_time_factor: 1.047498835
sensor {
  sensor_name: "world_1::model_1::link_1::sensor_contact"
  real_sensor_update_rate: 1048.5675518673938
  sim_sensor_update_rate: 1000
}
```

### max step size

The maximum time step size that can be taken by a variable time-step solver (such as simbody) during simulation.
For physics engines with fixed-step solvers (like ODE), this is simply the time step size. The default value
in Gazebo is 0.001 seconds.

When you choose the `update rate` of a sensor you need to take in account if you have enough precision
defined in `max step size`. For example:

  - 1/250 = 0.004 it's fine using the default value.
  - 1/400 = 0.0025. You will need to reduce the `max step size` to 0.0001, this will make the simulation slower. Change this
  parameter has a side effect on the accuracy and speed of physics simulation, refer to this [tutorial](http://gazebosim.org/tutorials?tut=physics_params&cat=physics) for more information.

In the following traces we included a IMU sensor at 400Hz. You can see that the `sim_sensor_update_rate` does not correspond with the defined value
because the `max step size` does not have enough precision.

```bash
sensor {
  sensor_name: "world_1::model_1::link_1::sensor_contact"
  real_sensor_update_rate: 181.87525758083353
  sim_sensor_update_rate: 200
}
...
sensor {
  sensor_name: "world_1::model_1::link_1::sensor_contact"
  real_sensor_update_rate: 252.94164813236739
  sim_sensor_update_rate: 333.33333333333331
}
```

If we modify the `max step size` to 0.0001, then you will see the right value.

```bash
sensor {
  sensor_name: "world_1::model_1::link_1::sensor_contact"
  real_sensor_update_rate: 39.899370595408726
  sim_sensor_update_rate: 400
}
```
