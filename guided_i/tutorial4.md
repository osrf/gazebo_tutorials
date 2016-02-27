# Recap

The previous three tutorials have led us through the process of creating
a sensor model, contributing the sensor to an online database, and improving
the model's visual appearance. This tutorial will improve the output from
the sensor through the addition of noise.

# Sensor Noise

Every, or nearly every, sensor has noise in the output. Cameras can have
chromatic abberations, sonars multi-path effects, and lasers incorrect
distance readings. In order to more closely match the type of data a real
sensors generates, we have to add noise to the data generated in simulation.

Gazebo has a built in noise model that can apply Gaussian noise to a variety
of sensors. While Gaussian noise may not be super realistic, it is better
than nothing and serves as a good first-pass approximation of noise.
Gaussian noise is also relatively easy to apply to data streams.

For more information on Gazebo's sensor noise model, visit [this tutorial](http://gazebosim.org/tutorials?tut=sensor_noise&cat=sensors).

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

    1. Left click in fron the laser beams to place the box.

        [[file:files/box_no_noise.jpg|800px]]

1. We can get a closer look at the sensor data through Gazebo's topic
   visualizer.

    1. Press Ctrl-t, to open the topic selector. Find the
       `/gazebo/default/velodyne/top/sensor/scan` topic.

        [[file:files/topic_selector.png]]

    1. Select the `/gazebo/default/velodyne/top/sensor/scan` topic, and
       press Okay to open a laser visualizer.

        [[file:files/velodyne_vis_no_noise.png]]

    1. Notice the nice smooth arc of the outptut.

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

The final tutorial in this series will add a plugin to the Velodyne sensor.
This plugin will control the rotation of the sensor's upper portion.

[Control plugin](http://gazebosim.org/tutorials?cat=guided_i&tut=guided_i5)
