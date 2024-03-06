# Recap

The previous tutorials have led us through the process of creating a sensor
model, and improving the model's visual appearance. This tutorial will
improve the output from the sensor through the addition of noise.

# Sensor Noise

Every, or nearly every, sensor has noise in the output. Cameras can have
chromatic aberrations, sonars multi-path effects, and lasers incorrect
distance readings. We have to add noise to the data generated in simulation in order to more closely match the type of data a real sensor produces.

Gazebo has a built in noise model that can apply Gaussian noise to a variety
of sensors. While Gaussian noise may not be very realistic, it is better
than nothing and serves as a good first-pass approximation of noise.
Gaussian noise is also relatively easy to apply to data streams.

For more information on Gazebo's sensor noise model, visit [this tutorial](/tutorials?tut=sensor_noise&cat=sensors).

## Step 1: Visualize the sensor data

Let's start by looking at the current Velodyne output, and then we can add
noise.

1. Open Gazebo, and insert the Velodyne sensor.

    1. ```gazebo```
    1. Select the Insert Tab near the upper left.
    1. Scroll down and select the Velodyne HDL-32 model.
    1. Click on a location in the render window. 

1. Add a Box in front of the laser beams, so that we get useful data.

    1. Select the Box icon in the toolbar above the render window.

    1. Left click in front the laser beams to place the box.

        [[file:files/box_no_noise.jpg|800px]]

1. We can get a closer look at the sensor data through Gazebo's topic
   visualizer.

    1. Press Ctrl-t, to open the topic selector. Find the
       `/gazebo/default/velodyne/top/sensor/scan` topic.

        [[file:files/topic_selector.png]]

    1. Select the `/gazebo/default/velodyne/top/sensor/scan` topic, and
       press Okay to open a laser visualizer.

        [[file:files/velodyne_vis_no_noise.png]]

    1. Notice the nice smooth lines of the outptut.

## Step 2: Add noise to the sensor

Gazebo's noise model can be accessed using the `<noise>` tag. See
[sdformat.org/spec](http://sdformat.org/spec) for more information.

1. Open the Velodyne model.

    ```
    gedit ~/.gazebo/models/velodyne_hdl32/model.sdf
    ```

1. Add a `<noise>` element as a child of the `<ray>` element. We will apply
   a large amount of noise at first so that the effects are readily visible.

    ```
    <sensor type="ray" name="sensor">
      <pose>0 0 -0.004645 1.5707 0 0</pose>
      <visualize>true</visualize>
      <ray>
        <noise>
          <!-- Use gaussian noise -->
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.1</stddev>
        </noise>
    ```

1. Once again, add the Velodyne sensor to Gazebo, and insert a box in front
   of the beams. 

1. Open the topic visualizer (Ctrl-t), and select the Velodyne laser scan
   topic. The output should look very noisy.

    [[file:files/velodyne_noisy.png]]

1. Now let's reduce the noise to something reasonable.

    ```
    <sensor type="ray" name="sensor">
      <pose>0 0 -0.004645 1.5707 0 0</pose>
      <visualize>true</visualize>
      <ray>
        <noise>
          <!-- Use gaussian noise -->
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.02</stddev>
        </noise>
    ```

# Next up

In the next section we will modify the Velodyne model so that it can be
easily shared and reused.

[Next Section](/tutorials?cat=guided_i&tut=guided_i4)
