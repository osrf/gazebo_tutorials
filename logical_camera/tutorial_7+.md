# Logical Camera
A logical camera outputs the names and poses of models that might appear in a camera with the same parameters.

# Example of a Logical Camera
Lets see what a logical camera does.
Download and save [this world.](http://bitbucket.org/osrf/gazebo_tutorials/raw/default/logical_camera/files/tutorial_logical_camera.world)

<include from='/#include/' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/logical_camera/files/tutorial_logical_camera.world'/>

Launch the downloaded world:

```
gazebo --verbose ./tutorial_logical_camera.world
```

From the top menu click `Window` then `Topic Visualization`.
Select the topic `/gazebo/default/post/link/logical_camera/models`.
It has the message type `gazebo.msgs.LogicalCameraImage`.

You should see the following:

[[file:files/tutorial_logical_camera.world.png|480px]]

#SDFormat Parameters
* `<update_rate>`
  This is a value in Hz that says how often in simulated time the logical camera will generate sensor data.
* `<topic>`
  The gazebo transport topic to publish to.
* `<pose>`
  If the model, link, and sensor poses are all zero, the world positive X axis will point out the center of the logical camera.
  The pose of a `<sensor>` is relative to the pose of the `<link>` which is relative to the pose of the `<model>`.
* `<visualize>`
  If true then the camera's frustum will be visualized in the gazebo client.
* `<logical_camera><near>`
  Distance in meters from the pose of the sensor to the closest point on the near clip plane.
* `<logical_camera><far>`
  Distance in meters from the pose of the sensor to the closest point on the far clip plane.
* `<logical_camera><horizontal_fov>`
  The horizontal field of view of the camera in radians.
* `<logical_camera><aspect_ratio>`
  The ratio of the width and height of the camera.
  The aspect ratio combined with the horizontal field of view defines the vertical field of view of the camera.

# Getting Data
There are two ways to get the data: directly from the sensor, or using gazebo transport.
Both ways return a [LogicalCameraImage](https://bitbucket.org/osrf/gazebo/src/gazebo7/gazebo/msgs/logical_camera_image.proto).

## Get Directly From The Sensor
To get data directly from the sensor you'll need to write a [gazebo plugin](http://gazebosim.org/tutorials?tut=plugins_hello_world&cat=write_plugin).

1. First get a generic sensor pointer.
  You'll need the name of the sensor.
  
      gazebo::sensors::SensorPtr genericSensor = sensors::get_sensor("model_name::link_name::my_logical_camera")

2. Cast it to a [LogicalCameraSensor](http://osrf-distributions.s3.amazonaws.com/gazebo/api/7.1.0/classgazebo_1_1sensors_1_1LogicalCameraSensor.html).
  
      sensors::LogicalCameraSensorPtr logicalCamera = std::dynamic_pointer_cast<sensors::LogicalCameraSensor>(genericSensor);

3. Finaly call [LogicalCamera::Image()](http://osrf-distributions.s3.amazonaws.com/gazebo/api/7.1.0/classgazebo_1_1sensors_1_1LogicalCameraSensor.html#a753f458d95c8f7abcfa87b19fffe0021) to get the latest sensor reading.
  
      gazebo::msgs::LogicalCameraImage sensorOutput = logicalCamera->Image();

## Get Using Gazebo Transport
// TODO
// What topic to subscribe to?
// What message to use?
// How to make a subscriber?

# Data Explanation
The logical camera reports the names and poses of models it sees.

While a normal camera sees `<visual>` geometry, the logical camera sees `<collision>`.
It works by testing if its [frustum](https://en.wikipedia.org/wiki/Viewing_frustum) intersects the [axis aligned bounding box](https://en.wikipedia.org/wiki/Bounding_volume) (**AABB**) of a model.
The model's AABB is just big enough to contain all of its `<collision>` geometry.
The AABB of a model does not contain nested models.

## Model Names
Model names reported by the camera are scoped names.
A model in the world `<model name="robot">` has a scoped named `robot`.
If it had a nested model `<model name="hand">`, the nested model's scoped name would be `robot::hand`.

These can be used to get a pointer to the model.

```
// Get the world (named "default")
gazebo::physics::WorldPtr world = physics::get_world("default");

// Get a model by name
gazebo::physics::ModelPtr handModel = world->GetModel("robot::hand");
```

## Model Poses
The logical camera outputs poses of models it sees relative to itself.
Included in the message is the pose of the camera itself.
To get the poses in world coordinates, combine it with the pose of the camera.

// TODO example

**Note:**
*The pose of a model is a fixed transform from the first link, called the canonical link.*

## False Positives
The output of a logical camera may include models a normal camera would not see if:

* the model has no `<visual>`
* the `<visual>` are transparent
* the model is rotated in the world such that its AABB is in the frustum while its geometry is not
* the `<collision>` on the model are larger or in different places than the `<visual>`
* the model is occluded by another model

## False Negatives
The output of a logical camera may not include models a normal camera would see if:

* the `<visual>` geometry is larger or in different places than the `<collision>`
* the `<sensor>` is part of the model

The logical camera never includes the model it is attached to in its output.
However, it will include any nested or parent models of the model it is attached to.
