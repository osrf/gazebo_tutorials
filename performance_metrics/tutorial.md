# Performance metrics

Gazebo 9 and Gazebo 11 publish a message called `/gazebo/performance_metrics` that allows to check the performance
of each sensor in the world.

# Learn how to read the measures

We are going to compare the results between physics (contact) and rendering (camera) based sensors and the differences
when the `lockstep` flag is active.

## Camera

For the rendering-based sensor we are going to use the [camera_strict_rate.world](https://github.com/osrf/gazebo/blob/gazebo9/test/worlds/camera_strict_rate.world) world. It contains two cameras: a high-resolution (1280x720) high-frame-rate (500 fps) and a a low-resolution (320x240) low-frame-rate (30 fps) camera.

### Run the world without lockstep

With the `lockstep` flag deactivated there is no guarantee when the callback is called. This means the `sim_sensor_update_rate` could not correspond with the real value defined in the world. Examine the `sim_sensor_update_rate` field. It is likely unable to reach the specified 500 fps. The other low-resolution camera is able to reach the specified value. Take note of the real time factor it should be close to 1.0.

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

### Run the world without lockstep

With the `lockstep` flag activated the `sim_sensor_update_rate` must correspond with the real value
defined in the world. In this case we expect a `sim_sensor_update_rate` equals to 500 in the high resolution camera.
Then the `fps` value means that Gazebo is able to generate 337 frames in a *real second*.

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

### Real time udpate rate

#### Run the world with lockstep

We can modify the Real time udpate rate in the Physics Engine. As you may see the `sim_sensor_update_rate` in both cameras
are the expected value 500 and 30. In this case the Real time udpate rate is equal to 10 which make the simulation slower. The fps
are lower, it's able to generate almost 6 frames for the high speed camera and 2 for the regular camera each *real second*.

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

Again in this case, where we run the world without `lockstep`, there is no guarantee when the callback is called, and the `sim_sensor_update_rate` value is highly impacted, it's should be close to 500 but in this case it's close to 0.

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

## Contact sensor

For the physics-based sensors we are going to use the [contact_strict_rate.world](https://github.com/osrf/gazebo/blob/gazebo9/test/worlds/contact_strict_rate.world) world. It contains a contact sensor (1000 Hz). To explain what is happening with
physics-based sensors we are going to use the `profiler`. When the `--lockstep` is activated,
the rendering-based sensors block the main thread until it gets the reading from the sensors.
We are going to change the `real_sensor_update_rate` to understand better what is happening and
this could affect the performance metrics:

 - real_sensor_update_rate equals 1000
[[file:files/camera_1000.png|600px]]
- real_sensor_update_rate equals 100
[[file:files/camera_100.png|600px]]
- real_sensor_update_rate equals 10
[[file:files/camera_10.png|600px]]

As you can see the `Server::Run` takes different times to execute depending on the real_sensor_update_rate to accomplish the desired rate.

However the physics-based sensors run in their own threads (separate from physics, rendering, and rendering sensor threads).

- real_sensor_update_rate equals 1000
[[file:files/contact_1000.png|600px]]
- real_sensor_update_rate equals 100
[[file:files/contact_100.png|600px]]
- real_sensor_update_rate equals 10
[[file:files/contact_10.png|600px]]

In this case, this type of sensor are not blocking the main thread which means that `Server::Run` will use always the same time (~100ms). This means that we are going to miss some reading.

### Run the world without lockstep

If `lockstep` is disabled then `Server::Run` will run as fast as possible, we are not going to miss any reading, we will see always a `sim_sensor_update_rate` closed to 1000 even if we change the `real_sensor_update_rate`.

```bash
real_time_factor: 0.994995308
sensor {
  sensor_name: "default::model_1::link_1::sensor_contact"
  real_sensor_update_rate: 900.55041641451237
  sim_sensor_update_rate: 1000
}
```

### Run the world with lockstep

If `lockstep` is enabled the `Server::Run` will run at 100ms if there is no a rendering-based sensor in the world that may change this value as explained before.

- real_sensor_update_rate equals 1000

```bash
real_time_factor: 0.994626914
sensor {
  sensor_name: "default::model_1::link_1::sensor_contact"
  real_sensor_update_rate: 9.98317126836121
  sim_sensor_update_rate: 10.1010101010101
}
```

- real_sensor_update_rate equals 100
```bash
real_time_factor: 0.994626914
sensor {
  sensor_name: "default::model_1::link_1::sensor_contact"
  real_sensor_update_rate: 9.98317126836121
  sim_sensor_update_rate: 100.1010101010101
}
```

- real_sensor_update_rate equals 10
```bash
real_time_factor: 0.994626914
sensor {
  sensor_name: "default::model_1::link_1::sensor_contact"
  real_sensor_update_rate: 9.98317126836121
  sim_sensor_update_rate: 1000.1010101010101
}
```

## Mix both world

If we include the contact model in the [camera_strict_rate.world](https://github.com/osrf/gazebo/blob/gazebo9/test/worlds/camera_strict_rate.world).
We will see that the `sim_sensor_update_rate` for the contact sensor is 500 and not 10 because the high speed camera it's blocking the main thread,
in this case we will miss only the half of the readings from the contact sensor.

```bash
real_time_factor: 0.61195177500000009
sensor {
  sensor_name: "default::camera_model::link::camera_sensor"
  real_sensor_update_rate: 336.56876890885411
  sim_sensor_update_rate: 500
  fps: 314.1287841796875
}
sensor {
  sensor_name: "default::camera_model::link::camera_sensor_regular"
  real_sensor_update_rate: 19.435832211237955
  sim_sensor_update_rate: 29.411764705882351
  fps: 18.927814483642578
}
sensor {
  sensor_name: "default::model_1::link_1::sensor_contact"
  real_sensor_update_rate: 336.4000684910539
  sim_sensor_update_rate: 500
}
```
