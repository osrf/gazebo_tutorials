# Logical Camera
While a camera outputs an image, a logical camera outputs model names and poses.
It shows which models might be visible to a camera in the same location.

# Example of a Logical Camera
1. Download and save [this world.](http://github.com/osrf/gazebo_tutorials/raw/master/logical_camera_sensor/files/tutorial_logical_camera.world)
    
    <include lang='xml' from='/#include/' src='http://github.com/osrf/gazebo_tutorials/raw/master/logical_camera_sensor/files/tutorial_logical_camera.world'/>

1. Launch the world
        
        gazebo --verbose ./tutorial_logical_camera.world

1. From the top menu click **Window** then **Topic Visualization**.
1. Select the topic **/gazebo/default/post/link/logical_camera/models**.
    * It will be listed under the message type **gazebo.msgs.LogicalCameraImage**.
1. You should see the following:
    
    [[file:files/tutorial_logical_camera.world.png|800px]]


# Data Explanation
The logical camera reports the names and poses of models it sees.

While a normal camera sees `<visual>` geometry, the logical camera sees `<collision>`.
It works by testing if its [frustum](https://en.wikipedia.org/wiki/Viewing_frustum) intersects the [axis aligned bounding box](https://en.wikipedia.org/wiki/Bounding_volume) (**AABB**) of a model.
The model's AABB is just big enough to contain all of its `<collision>` geometry.


## Model Names
Model names reported by the camera are scoped names.
A model in the world `<model name="robot">` has a scoped named `robot`.
If it had a nested model `<model name="hand">`, the nested model's scoped name would be `robot::hand`.

The scoped name can be used to get a pointer to the model.

```cpp
// Get the world (named "default")
gazebo::physics::WorldPtr world = physics::get_world("default");

// Get a model by name
gazebo::physics::ModelPtr handModel = world->GetModel("robot::hand");
```

## Model Poses
The logical camera outputs poses of models it sees relative to itself.
It also outputs its own pose in world coordinates.
Use this to get the model poses in world coordinates.

```cpp
// msg is of type gazebo::msgs::LogicalCameraImage

// Pose of camera in world coordinates
ignition::math::Pose3d cameraPose = gazebo::msgs::ConvertIgn(msg.pose());

// Pose of first model relative to camera
ignition::math::Pose3d modelRelativePose = gazebo::msgs::ConvertIgn(msg.model(0).pose());

// Pose of first model in world coordinates
ignition::math::Pose3d modelWorldPose = modelRelativePose - cameraPose;
```

## False Positives
The output of a logical camera may include a model a normal camera would not see if:

* The model has no `<visual>`.
* The `<visual>` are transparent.
* The `<collision>` on the model are larger or in different places than the `<visual>`.
* The model is occluded by another model.
* The model is rotated in the world such that its AABB is in the frustum while the geometry is not.
  Since the volume of a model's AABB is greater than or equal to the volume of the collisions on it,
  it is possible some part of this extra volume intersects the frustum while none of the collision geometry does.
  This is illustrated in the following image:

[[file:files/tutorial_logical_camera_aabb_false_positive.world.png|600px]]

## False Negatives
The output of a logical camera may not include a model a normal camera would see if:

* The `<visual>` geometry is larger or in different places than the `<collision>`
* The logical camera is on the model.
  The logical camera never includes the model it is part of in its output.
  However, it will include any nested or parent models.
* The only part of a model within the frustum is a nested model.
  A model's AABB does not contain its nested models.
  Every nested model has its own AABB.
  If only the nested model is visible then only it will be in the output.

#SDFormat Parameters
* `<update_rate>`
  * Number of times per simulated second the sensor will generate data.
* `<topic>`
  * The gazebo transport topic to publish to.
    If this is not specified then the sensor will generate a topic name that includes the names of its ancestors in SDF.
* `<pose>`
  * If the model, link, and sensor poses are all zero, the world positive X axis will point out the center of the logical camera.
* `<visualize>`
  * If true then the camera's frustum will be visualized in the gazebo client.
* `<logical_camera>`
  * `<near>`
      * Distance in meters from the pose of the sensor to the closest point on the near clip plane.
  * `<far>`
      * Distance in meters from the pose of the sensor to the closest point on the far clip plane.
  * `<horizontal_fov>`
      * The horizontal field of view of the camera in radians.
  * `<aspect_ratio>`
      * The ratio of the width and height of the camera.
        The aspect ratio combined with the horizontal field of view defines the vertical field of view of the camera.

# Getting Data
There are two ways to get the data: directly from the sensor, or using gazebo transport.
Both ways return a [LogicalCameraImage](https://github.com/osrf/gazebo/blob/gazebo7/gazebo/msgs/logical_camera_image.proto).

## Directly From The Sensor
The data can be taken directly from the sensor.
You will need to write a [gazebo plugin](/tutorials?tut=plugins_hello_world&cat=write_plugin) to access it.
Only the most recently generated message is available.

1. Get a generic sensor pointer using the name of the sensor.
    
    ~~~cpp
    gazebo::sensors::SensorPtr genericSensor = sensors::get_sensor("model_name::link_name::my_logical_camera")
    ~~~

1. Cast it to a [LogicalCameraSensor](http://osrf-distributions.s3.amazonaws.com/gazebo/api/7.1.0/classgazebo_1_1sensors_1_1LogicalCameraSensor.html).
    
    ~~~cpp
    sensors::LogicalCameraSensorPtr logicalCamera = std::dynamic_pointer_cast<sensors::LogicalCameraSensor>(genericSensor);
    ~~~

1. Call [LogicalCamera::Image()](http://osrf-distributions.s3.amazonaws.com/gazebo/api/7.1.0/classgazebo_1_1sensors_1_1LogicalCameraSensor.html#a753f458d95c8f7abcfa87b19fffe0021) to get the latest sensor data.
    
    ~~~cpp
    gazebo::msgs::LogicalCameraImage sensorOutput = logicalCamera->Image();
    ~~~

## Using Gazebo Transport
Logical camera sensor data is published using gazebo transport.
The data is published to subscribers immediately after it is generated.

1. Create a gazebo transport node and initialize it.
    
    ~~~cpp
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();
    ~~~

1. Create a callback for the subscription
    
    ~~~cpp
    /////////////////////////////////////////////////
    // Function is called everytime a message is received.
    void callback(gazebo::msgs::ConstLogicalCameraImagePtr &_msg)
    {
      // your code here
    }
    ~~~

1. Listen to the topic published by the logical camera
    
    ~~~cpp
    gazebo::transport::SubscriberPtr sub = node->Subscribe("~/post/link/logical_camera/models", callback);
    ~~~
