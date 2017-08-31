# Logical Camera

// What is a logical camera
// How is it different from a regular camera

A logical camera reports the names and relative poses of objects it can see.
It is useful when one wants to know what objects might be seen by a camera with the same parameters.

# Adding a Logical Camera to a Model

// Example with a camera on a model

#SDFormat Parameters
`<update_rate>`
This is a value in Hz that says how often in simulated time the logical camera will generate sensor data.
`<topic>`
TODO is this used?
`<pose>`
If the model, link, and sensor `<pose>` of the logical camera were all zero, the world positive X axis would point out the center of the camera.
The pose of the `<sensor>` is relative to the pose of the `<link>`, which itself is relative to the pose of the `<model>`.

`<visualize>`
If true then lines will be drawn to show the logical camera's frustum.

`<logical_camera><near>`
Distance in meters from the pose of the sensor to the closest point on the near clip plane.
`<logical_camera><far>`
Distance in meters from the pose of the sensor to the furthest point on the near clip plane.
`<logical_camera><horizontal_fov>`
The horizontal field of view of the camera in radians.
`<logical_camera><aspect_ratio>`
The ratio of the width and height of the camera.
The aspect ratio combined with the horizontal field of view defines the vertical field of view of the camera.

# Getting Logical Camera Data
// Two ways: programatically, and gazebo transport
// High level what kind of data is it?

## Getting the Data via C++
// What does the data look like?
// How to get the camera on a model?

## Getting the Data via Gazebo Transport
// What topic to subscribe to?
// What message to use?
// How to make a subscriber?

# Explanation of the Data
The logical reports the names of models whose axis aligned bounding boxes intersect with its frustum.

## Models Seen by Logical Camera but not a Real Camera
There are some cases where logical camera reports an object that a real camera would not see.
For example, if the logical camera is pointed a wall and a construction cone is behind the wall the output would include both.
// TODO Image of wall and construction cone
A real camera would only show an image of the wall.
Another example happens depending on how the model is rotated in the world.
The model's bounding box is aligned to the world so it is possible for it to insersect the frustum depending on the orientation of the camera and model in the world.
// TODO image of camera/box rotated showing the AABB intersecting with the frustum

## Poses
The poses reported are relative to the logical camera's pose.
// TODO more about poses here
// Can get the world pose of the models by combine the sensor pose with the model poses in the message
// Orientation reflects the rotation of the model, not of the links


// What are the names?
// Nested model names?
//  Not detected #2342
// How can the names be used?
//  physics::WorldPtr->GetModel(modelName);
// How much of a model needs to show for it to be visible?
//  The axis aligned bounding box needs to overlap with the frustum
//    TODO Is axis aligned bounding box size driven by visuals or collisions?
// Where are the poses located?
//   Relative to the sensor
// How to interpret the visualized data?
//   Near and far clip planes
//   FOV
