#Prerequisites

  [Overview / HelloWorld](http://gazebosim.org/tutorials?tut=plugins_hello_world) Plugin Tutorial

# Overview

This tutorial explains how to use a model plugin for gravity compensation in Gazebo as well as how it complements the built-in PID joint controllers.

Gravity compensation is a technique used to mitigate the effects of gravity on a robot's behavior. A model of the robot and its current configuration are used to estimate the gravitational forces acting on the robot's links and the joint efforts necessary to balance them. Ideally, gravity compensation would cancel out accelerations due to gravity while allowing the robot to comply with other external forces.

We will begin by controlling a 0.1 kg mass on a linear actuator using PID feedback and then see how gravity compensation can improve control.

# PID Control
## P Control

1. Start Gazebo with the simulation paused (-u option), and insert the `Mass on rails` model.

[[file:files/mass_on_rails.png|800px]]

2. Select the model and drag open the right panel to reveal the joint control interface.

3. Open the `Position` tab and set the target position to `1.0`, the proportional gain to `2.0`, and the other gains to `0.0` as shown below.

[[file:files/joint_control_gui.png|800px]]

4. Unpause the simulation.

5. Once the mass comes to rest due to actuator friction in the model, check the `pose` property of the `mass` link in the left panel.

The mass has moved but it has fallen far short of the target position. If you increase the gain, the mass will stop closer to the target. However, increasing the gain also increase the amplitude of the oscillations of the mass and time it takes to settle as illustrated below.

[[file:files/p_control.png|600px]]

## Plotting

To create your own plot similar to the one above:

* Make sure the simulation is paused and the time reset to zero.

* Click `Plot` under the `Window` menu.

* Select the `MODELS` tab.

* Expand the model tree to find the position of the mass: `mass_on_rails` > `mass` > `Pose` > `Position`

* Click and drag the `Z` element onto the plot canvas.

* Set the joint controller parameters and unpause the simulation.

To add new traces to the plot for different parameters:

* Pause the simulation.

* Reset the simulation (`Ctrl-R`).

* Set the new joint controller gains and unpause the simulation.

[[file:files/plot_example.png|600px]]

## PD Control

The oscillations can be eliminated while keeping the proportional gain high by using the derivative gain. While the proportional term opposes the controller error (in this case, position) the derivative term opposes the rate of change of the controller error (in this case, velocity). The derivative term provides linear damping.

Neglecting friction, actuator dynamics, etc., the example system with PD control can be modeled as a [damped harmonic oscillator](https://en.wikipedia.org/wiki/Harmonic_oscillator#Damped_harmonic_oscillator). The proportional and derivative gains correspond to the variables "k" and "c" respectively on the linked page.

We can choose the natural frequency and damping ratio of the system by setting the gains appropriately. Let us pick a damping ratio of 1 and a natural frequency of 10 rad/s. The corresponding proportional and derivative gains are 10 and 2 given a 0.1 kg mass.

Larger values for the natural frequency generally produce faster convergence toward equilibrium and more rapid oscillations if the system is underdamped. Larger values for the damping ratio reduce and eventually eliminate oscillations, but a damping ratio that is too large or small will slow convergence. A damping ratio of 1 produces critical damping: the fastest convergence rate possible without oscillations for a given natural frequency.

5. Set the proportional and derivative gains to `10.0` and `2.0` respectively.

6. Reset the simulation (`Ctrl-R`).

The mass now settles near the target position relatively quickly. However, there is still room for improvement as some steady-state error can be observed.

## PID Control

An integral term in the controller can eliminate steady-state error. The integral term responds to the controller error summed over time, so it will increase in magnitude until the steady-state error is eliminated or it reaches some threshold (currently +-1 in Gazebo).

7. Set the proportional, integral, and derivative gains to `10.0`, `1.0`, and `2.0` respectively and reset the simulation again.

The mass now stops nearly exactly at the target. The plot below shows the position of the mass over time for both the PD and PID control examples.

[[file:files/pid_control.png|600px]]

Care must be taken in tuning PID controllers. For example, if we set the PID gains to 10.0, 1.0, and 0.0 instead of the above, the mass oscillates wildly.

# Gravity Compensation

The PID controller now performs well for the `mass_on_rails` system. However, it relies on high gains which may be undesirable, for example, if we want compliance in controller. In some situations, gravity compensation can provide a means to reduce PID gains without completely sacrificing the controller performance.

## Demonstration
For the `mass_on_rails` system, gravity is the primary factor the controller must overcome. So, if we add gravity compensation (GC), we can reduce the PID gains and still have the mass settle near the target.

1. Open the example world in Gazebo with the simulation paused.

~~~
gazebo -u --verbose worlds/gravity_compensation.world
~~~

2. Set the target position to `1.0` and set the proportional, integral, and derivative gains to `0.4`, `0.0`, and `0.4` respectively.

3. Unpause the simulation.

With gravity compensation, the mass settles near the target despite lower PID gains. The controller has some steady-state error because the integral term is zero and the model includes friction.

[[file:files/gravity_compensation.png|600px]]

## Using the Plugin

The following excerpt from `gravity_compensation.world` shows how it calls the  plugin:

~~~
<include>
  <uri>model://mass_on_rails</uri>
  <plugin name="gravity_compensation" filename="libGravityCompensationPlugin.so">
    <uri>model://mass_on_rails</uri>
  </plugin>
</include>
~~~

The `<include>` block tells Gazebo to add a model to the world, and the URI indicates which model to add. The `<plugin>` block tells Gazebo to run the gravity compensation plugin for this model (the outer block). The `<plugin>` block requires a model URI of its own, which identifies the model to use when calculating the forces due to gravity and the compensating joint efforts.

> *Note 1*: A model could be defined directly in the world file by replacing the `<include>` block with a `<model>` block and adding the necessary attributes and elements.

> *Note 2*: The URIs in the `<include>` and `<plugin>` blocks of the world file need not match as long as the `<plugin>` model includes all the joints in the `<include>` model.

## Model Error

When applied to a physical robot, gravity compensation may exhibit varying degrees of error depending on the discrepancy between the model and physical robot. The ability to use different models for the plugin and simulation is useful for studying the effects of model error. For example, if we anticipate that the real mass may differ from the model, we could update the simulation's model while using the old model in the plugin.

4. Right click on the model and select `Edit Model`.

5. In the left pane under the `Model` tab, double click the `mass` link and set its mass to `0.09`.

6. Exit the model editor, saving the model when prompted.

7. Select the model and enter the joint controller parameters again..

8. Unpause the simulation.

The controller overshoots the target by a large margin because it weighs more in the plugin's model. So, if error of 0.01kg in the mass estimate is realistic, we might consider adding back an integral term and/or increasing the PID gains.

[[file:files/model_error.png|600px]]

## Robonaut

Gravity compensation can readily extend to more complex systems. The GIF below shows Robonaut 2 models with and without gravity compensation. The left Robonaut holds its pose with gravity compensation while the right one lets its arms drop to its sides.

[[file:files/robonaut.gif|400px]]

To run this example yourself, start gazebo with the following world file:

~~~
<?xml version="1.0"?>
<sdf version="1.5">
  <world name="default">
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>
    <include>
      <uri>model://r2_description</uri>
      <name>r2_grav_comp</name>
      <pose>0 -2 0 0 0 0</pose>
      <plugin name="gravity_compensation" filename="libGravityCompensationPlugin.so">
        <uri>model://r2_description</uri>
      </plugin>
    </include>
    <include>
      <uri>model://r2_description</uri>
      <pose>2 0 0 0 0 -1.57</pose>
    </include>
  </world>
</sdf>
~~~
