# Tutorial: ROS Advanced Integration

## Dynamic Reconfigure

Some of the physics properties can be adjusted within Gazebo as we described in the [http://gazebosim.org/wiki/Tutorials/1.9/make_world ''modifying a world tutorial'']. In addition, we can modify this properties using ROS's dynamic reconfigure mechanism.

As an example, we'll invert the gravity in the simulation. Make sure you have the following installed for groovy:

<pre>
sudo apt-get install ros-groovy-rqt ros-groovy-dynamic-reconfigure
</pre>

Or these packages for hydro:

<pre>
sudo apt-get install ros-hydro-rqt ros-hydro-dynamic-reconfigure
</pre>

Start Gazebo:

<pre>
rosrun gazebo_ros gazebo
</pre>

Let's insert a model in gazebo before making any change in the physics. Go ahead and click ''Insert'' in the left side of the gazebo main window. Click on the ''Pioneer 2DX'' robot model.

Start RosGui tool for interacting with gazebo in runtime:

<pre>
rosrun rqt_gui rqt_gui
</pre>


[[file:RosGUI.png|600px]]

Resize the RosGui tool until you see something similar to the upper picture. Click on ''gazebo'' in the left side of RosGui. You will see the physics parameter list that you will be able to modify. Change the ''gravity_z'' parameter to ''+9.8'' and you should see how the gravity affects your robot.

[[file:pioneer_flying.png|600px]]

