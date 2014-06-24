# Tutorial: ROS Control

In this tutorial we will setup simulated controllers to actuate the joints of your robot. This will allow us to provide the correct ROS interfaces for planners like [http://moveit.ros.org/ MoveIt!]. We will be using the [http://ros.org/wiki/ros_control ros_control] packages, a new standard in ROS for controller interfaces.

## About ros_control 

We encourage you to read an overview of the documentation on [http://ros.org/wiki/ros_control ros_control] before proceeding.

## Data flow of ros_control and Gazebo 

Simulating a robot's controllers in Gazebo can be accomplished using ros_control and a simple Gazebo plugin adapter. An overview of the relationship between simulation, hardware, controllers and transmissions is shown below:

[[file:Gazebo_ros_transmission.png|800px]]

# Prerequisites 

This tutorial builds off of many of the concepts in the previous tutorials. We will again be using the RRBot that was setup in the [Using URDF in Gazebo](http://gazebosim.org/tutorials/?tut=ros_urdf) tutorial, as an example for the plugins covered here.

Make sure you have already installed <tt>ros_control</tt>, <tt>ros_controllers</tt>, and their dependencies as described in the [installation instructions](http://gazebosim.org/wiki/Tutorials/1.9/Installing_gazebo_ros_Packages).

# Usage

## Add transmission elements to a URDF

To use ros_control with your robot, you need to add some additional elements to your URDF. The <tt><transmission></tt> element is used to link actuators to joints, see the [http://ros.org/wiki/urdf/XML/Transmission <transmission> spec] for exact XML format.

For the purposes of gazebo_ros_control in its current implementation, the only important information in these transmission tags are:

 * <joint name=""> - the name must correspond to a joint else where in your URDF
 * <type> - the type of transmission. Currently only "transmission_interface/SimpleTransmission" is implemented. (feel free to add more)
 * <hardwareInterface> - within the <actuator> tag, this tells the gazebo_ros_control plugin what hardware interface to load (position, velocity or effort interfaces). Currently only effort interfaces are implemented. (feel free to add more)

The rest of the names and elements are currently ignored.

## Add the gazebo_ros_control plugin

In addition to the transmission tags, a Gazebo plugin needs to be added to your URDF that actually parses the transmission tags and loads the appropriate hardware interfaces and controller manager. By default the gazebo_ros_control plugin is very simple, though it is also extensible via an additional plugin architecture to allow power users to create their own custom robot hardware interfaces between ros_control and Gazebo.

The default plugin XML should be added to your URDF:

~~~
<gazebo>
  <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
    <robotNamespace>/MYROBOT</robotNamespace>
  </plugin>
</gazebo>
~~~

The gazebo_ros_control <plugin> tag also has the following optional child elements:

 * <tt><robotNamespace></tt>: The ROS namespace to be used for this instance of the plugin, defaults to robot name in URDF/SDF
 * <tt><controlPeriod></tt>: The period of the controller update (in seconds), defaults to Gazebo's period
 * <tt><robotParam></tt>: The location of the robot_description (URDF) on the parameter server, defaults to '/robot_description'
 * <tt><robotSimType></tt>: The pluginlib name of a custom robot sim interface to be used (see below for more details), defaults to 'DefaultRobotHWSim'

### Default gazebo_ros_control Behavior

By default, without a <robotSimType> tag, gazebo_ros_control will attempt to get all of the information it needs to interface with a ros_control-based controller out of the URDF. This is sufficient for most cases, and good for at least getting started.

The default behavior provides the following ros_control interfaces:

 * hardware_interface::JointStateInterface
 * hardware_interface::EffortJointInterface
 * hardware_interface::VelocityJointInterface - *not fully implemented*

### Advanced: custom gazebo_ros_control Simulation Plugins

The gazebo_ros_control Gazebo plugin also provides a pluginlib-based interface to implement custom interfaces between Gazebo and ros_control for simulating more complex mechanisms (nonlinear springs, linkages, etc).

These plugins must inherit gazebo_ros_control::RobotHWSim which implements a simulated ros_control hardware_interface::RobotHW. RobotHWSim provides API-level access to read and command joint properties in the Gazebo simulator.

The respective RobotHWSim sub-class is specified in a URDF model and is loaded when the robot model is loaded. For example, the following XML will load the default plugin (same behavior as when using no <robotSimType> tag):

~~~
<gazebo>
  <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
    <robotNamespace>/MYROBOT</robotNamespace>
    <robotSimType>gazebo_ros_control/DefaultRobotHWSim</robotSimType>
  </plugin>
</gazebo>
~~~

## RRBot Example

We add a <transmission> block similar to the following for every joint that we wish to have Gazebo actuate. Open your <tt>rrbot.xacro</tt> file and at the bottom of the file you should see:

~~~
  <transmission name="tran1">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="joint1"/>
    <actuator name="motor1">
      <hardwareInterface>EffortJointInterface</hardwareInterface>
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>
  </transmission>

  <transmission name="tran2">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="joint2"/>
    <actuator name="motor2">
      <hardwareInterface>EffortJointInterface</hardwareInterface>
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>
  </transmission>
~~~

You'll also see the gazebo_ros_control plugin in <tt>rrbot.gazebo</tt> that reads in all the <transmission> tags:

~~~
<gazebo>
  <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
    <robotNamespace>/rrbot</robotNamespace>
  </plugin>
</gazebo>
~~~

## Create a ros_controls package

We'll next need to create a configuration file and launch file for our ros_control controllers that interface with Gazebo.

### Create new package

~~~
cd ~/catkin_ws
catkin_create_pkg MYROBOT_control ros_control ros_controllers
cd MYROBOT_control
mkdir config
mkdir launch
~~~

### Create a .yaml config file

The PID gains and controller settings must be saved in a yaml file that gets loaded to the param server via the roslaunch file. In the config folder of your <tt>MYROBOT_control</tt> package, adapt the following RRBot example to your robot:

~~~
rrbot:
  # Publish all joint states -----------------------------------
  joint_state_controller:
    type: joint_state_controller/JointStateController
    publish_rate: 50  
  
  # Position Controllers ---------------------------------------
  joint1_position_controller:
    type: effort_controllers/JointPositionController
    joint: joint1
    pid: {p: 100.0, i: 0.01, d: 10.0}
  joint2_position_controller:
    type: effort_controllers/JointPositionController
    joint: joint2
    pid: {p: 100.0, i: 0.01, d: 10.0}
~~~

See the next section for more details about these controllers.

### Create a roslaunch file

Create a roslaunch file for starting the ros_control controllers. Within the launch folder create a <tt>MYROBOT_control.launch</tt> file and adapt the following RRBot example to your robot:

~~~
<launch>

  <!-- Load joint controller configurations from YAML file to parameter server -->
  <rosparam file="$(find rrbot_control)/config/rrbot_control.yaml" command="load"/>

  <!-- load the controllers -->
  <node name="controller_spawner" pkg="controller_manager" type="spawner" respawn="false"
	output="screen" ns="/rrbot" args="joint1_position_controller joint2_position_controller joint_state_controller"/>

  <!-- convert joint states to TF transforms for rviz, etc -->
  <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher"
	respawn="false" output="screen">
    <remap from="/joint_states" to="/rrbot/joint_states" />
  </node>

</launch>
~~~

#### Explanation

The first line, "rosparam", loads the controller settings to the parameter server by loading a yaml configuration file (discussed in the next section).

The controller_spawner node starts the two joint position controllers for the RRBot by running a python script that makes a service call to the ros_control controller manager. The service calls tell the controller manager which controllers you want. It also loads a third controller that publishes the joint states of all the joints with hardware_interfaces and advertises the topic on /joint_states. The spawner is just a helper script for use with roslaunch. 

The final line starts a robot_state_publisher node that simply listens to /joint_states messages from the joint_state_controller then publishes the transforms to /tf. This allows you to see your simulated robot in Rviz as well as do other tasks.

## Start the controllers using roslaunch

Test the RRBot controlled by ros_control by running the following:

Start the RRBot simulation:

~~~
roslaunch rrbot_gazebo rrbot_world.launch
~~~

Load the controllers for the two joints by running the second launch file:

~~~
roslaunch rrbot_control rrbot_control.launch 
~~~

### Using service calls manually

If you first load the rrbot_control.yaml files to the parameter server, you could load the controllers manually through service requests. We'll include them here for reference though we usually prefer roslaunch:

Load the controllers:

~~~
rosservice call /rrbot/controller_manager/load_controller "name: 'joint1_position_controller'"
rosservice call /rrbot/controller_manager/load_controller "name: 'joint2_position_controller'"
~~~

Start the controllers:

~~~
rosservice call /rrbot/controller_manager/switch_controller "{start_controllers: ['joint1_position_controller','joint2_position_controller'], stop_controllers: [], strictness: 2}"
~~~

Stop the controllers:

~~~
rosservice call /rrbot/controller_manager/switch_controller "{start_controllers: [], stop_controllers: ['joint1_position_controller','joint2_position_controller'], strictness: 2}"
~~~

## Manually send example commands

Send example joint commands to them for testing:

~~~
rostopic pub -1 /rrbot/joint1_position_controller/command std_msgs/Float64 "data: 1.5"
rostopic pub -1 /rrbot/joint2_position_controller/command std_msgs/Float64 "data: 1.0"
~~~

## Use RQT To Send Commands

In this section we'll go over tools to help you visualize the performance of your controller and tune any gains/parameters the controller might have, particularly PID gains. We'll be using [www.ros.org/wiki/rqt RQT], ROS's plugin-based user interface, so be sure you first have that installed.

Start RQT:

~~~
rosrun rqt_gui rqt_gui
~~~

### Add a Command Publisher

Add the 'Publisher' plugin then choose the topic from the drop down box that commands any particular controller that you want to publish to. For the RRBot, add the controller:

~~~
/rrbot/joint1_position_controller/command
~~~

Then press the green plus sign button at the top right.

Enable the topic publisher by checking the check box on the left of the topic name. Set the rate column to 100 (the frequency we send it commands - 100hz in this case). 

Next, expand the topic so that you see the "data" row. In the expression column, on the data row, try different radian values between joint1's joint limits - in RRBot's case there are no limits because the joints are continuous, so any value works. You should be able to get the RRBot to swing around if you are doing this tutorial with that robot.

Next, in that same expression box we'll have it automatically change values using a sine wave. Add the following:

~~~
sin(i/100)
~~~

For more advanced control, you can configure it to publish a sine wave to your robot's exact joint limits:

~~~
sin(i/rate*speed)*diff + offset
~~~

An explanation of variables:

 * i - the RQT variable for time
 * rate - the frequency that this expression is evaluated. This should be the same number as in the rate column of the topic publisher. Recommended value is 100.
 * speed - how quick you want the join to actuate. Start off with just 1 for a slow speed
 * upper_limit and lower_limits - the joint limits of the hardware being controlled by this controller
 * diff = (upper_limit - lower_limit)/2 
 * offset = upper_limit-diff

### Visualize the controller's performance

Add a Plot plugin to RQT and add the same topic as the one you chose above for the topic publisher:

~~~
/rrbot/joint1_position_controller/command/data
~~~

Click the green add button. You should now see a sine wave being plotted on the screen.

Add another topic to the Plot plugin that tracks the error between the commanded position and actual position of the actuator being controlled. For the RRBot:

~~~
/rrbot/joint1_position_controller/state/error
~~~

You screen should look something like this:

[[file:rqt_controller_tuning.png|800px]]

Note: the RQT plot plugin is known to have bugs after running for a while (>1min). The drawings start acting strangely. The current solution is to press the blue refresh button at the top right of the plugin.

### Tune the PID gains

Finally, we'll use [http://ros.org/wiki/dynamic_reconfigure dynamic reconfigure] to tune the proportional, derivative, and integral gains of the PID controller, assuming this is applicable to your robot.

Add the 'Dynamic Reconfigure' plugin to RQT and click 'Expand All' to see the sub-options. Assuming your controller uses PID, you should use a "pid" option. Clicking on it should reveal 5 sliders that let you tune the controller, as pictured in the following screenshot. Adjust these values until you get the desired performance of your controller.

[[file:rqt_dynamnic_reconfigure_pid.png|600px]]

### Use roslaunch to save your RQT perspective

A pre-configured RQT perspective for the rrbot can be easily launched with the following command:

~~~
roslaunch rrbot_control rrbot_rqt.launch
~~~

You can use that as a template for doing this with your own robot.

## Connect Rviz to Gazebo Simulation

Now that you are using ros_control to send commands to your robot in simulation, you can also use the ros_control joint_state_controller to read the state of the robot from Gazebo. The idea behind a good simulator is that you should be able to use the same software on your real hardware as you do in simulation. A good starting point for that is visualizing your simulated robot in Rviz, similar to how it is done with real hardware.

Assuming you are already starting a joint_state_controller as documented above in your rosparam and roslaunch files, your next step is to start Rviz:

~~~
rosrun rviz rviz
~~~

Under "Global Options" change your "Fixed Frame" to "world" to resolve any errors it might be giving you.

Next, add a "RobotModel" display type to Rviz and you should then see your simulated robot in Gazebo being visualized in Rviz!

# Demo Code

The example code used for the RRBot in this tutorial is available in the repository [https://github.com/ros-simulation/gazebo_ros_demos gazebo_ros_demos].

# Next Steps

Learn about ROS message and service calls that are available for use with Gazebo in the tutorial [ROS Communication with Gazebo](http://gazebosim.org/tutorials/?tut=ros_comm).
