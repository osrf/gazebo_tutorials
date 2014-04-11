#Tutorial: Attach Gripper to Robot#

**Prerequisites:**

  [Make a Mobile Robot](http://gazebosim.org/tutorials/?tut=build_robot)

  [Make a Simple Gripper](http://gazebosim.org/tutorials/?tut=simple_gripper)

This tutorial explains how to create a composite robot from existing robot parts, i.e. mobile base, simple arm and simple gripper.

## Robot Components

Start up gazebo and make sure you can load the models from the two previous tutorials.

### Mobile Base

1. Per instructions in [Make a Mobile Robot](http://gazebosim.org/tutorials/?tut=build_robot) tutorial, you should have a mobile base robot at your disposal:

    <img src="http://gazebosim.org/w/images/0/0e/Mobile_base.png" width="640px"/>

1. For this exercise, modify `~/.gazebo/models/my_robot/model.sdf` to make the model larger so it can accommodate the gripper we are about to append to it:

        gedit ~/.gazebo/models/my_robot/model.sdf

    update the contents to make the model body larger and re-position the wheels accordingly:

    ~~~
    <?xml version='1.0'?>
    <sdf version='1.4'>
      <model name="mobile_base">
        <link name='chassis'>
          <pose>0 0 .25 0 0 0</pose>

          <inertial>
            <mass>20.0</mass>
            <pose>-0.1 0 -0.1 0 0 0</pose>
            <inertia>
              <ixx>0.5</ixx>
              <iyy>1.0</iyy>
              <izz>0.1</izz>
            </inertia>
          </inertial>

          <collision name='collision'>
            <geometry>
              <box>
                <size>2 1 0.3</size>
              </box>
            </geometry>
          </collision>

          <visual name='visual'>
            <geometry>
              <box>
                <size>2 1 0.3</size>
              </box>
            </geometry>
          </visual>

          <collision name='caster_collision'>
            <pose>-0.8 0 -0.125 0 0 0</pose>
            <geometry>
              <sphere>
                <radius>.125</radius>
              </sphere>
            </geometry>

            <surface>
              <friction>
                <ode>
                  <mu>0</mu>
                  <mu2>0</mu2>
                </ode>
              </friction>
            </surface>
          </collision>

          <visual name='caster_visual'>
            <pose>-0.8 0 -0.125 0 0 0</pose>
            <geometry>
              <sphere>
                <radius>.125</radius>
              </sphere>
            </geometry>
          </visual>
        </link>
        <link name="left_wheel">
          <pose>0.8 0.6 0.125 0 1.5707 1.5707</pose>
          <collision name="collision">
            <geometry>
              <cylinder>
                <radius>.125</radius>
                <length>.05</length>
              </cylinder>
            </geometry>
          </collision>
          <visual name="visual">
            <geometry>
              <cylinder>
                <radius>.125</radius>
                <length>.05</length>
              </cylinder>
            </geometry>
          </visual>
        </link>

        <link name="right_wheel">
          <pose>0.8 -0.6 0.125 0 1.5707 1.5707</pose>
          <collision name="collision">
            <geometry>
              <cylinder>
                <radius>.125</radius>
                <length>.05</length>
              </cylinder>
            </geometry>
          </collision>
          <visual name="visual">
            <geometry>
              <cylinder>
                <radius>.125</radius>
                <length>.05</length>
              </cylinder>
            </geometry>
          </visual>
        </link>

        <joint type="revolute" name="left_wheel_hinge">
          <pose>0 0 -0.03 0 0 0</pose>
          <child>left_wheel</child>
          <parent>chassis</parent>
          <axis>
            <xyz>0 1 0</xyz>
          </axis>
        </joint>

        <joint type="revolute" name="right_wheel_hinge">
          <pose>0 0 0.03 0 0 0</pose>
          <child>right_wheel</child>
          <parent>chassis</parent>
          <axis>
            <xyz>0 1 0</xyz>
          </axis>
        </joint>

      </model>
     </sdf>
    ~~~

    <img src="http://gazebosim.org/w/images/0/00/Mobile_base_large.png" width="640px"/>

### Assembling a Composite Robot

1. To created a mobile robot with a simple gripper attached, create a new models directory

        mkdir ~/.gazebo/models/simple_mobile_manipulator
        cd ~/.gazebo/models/simple_mobile_manipulator

    And edit the model config file:

        gedit ~/.gazebo/models/simple_mobile_manipulator/model.config

    populate it with the following contents:

    ~~~
    <?xml version="1.0"?>
    <model>
      <name>Simple Mobile Manipulator</name>
      <version>1.0</version>
      <sdf version='1.4'>model.sdf</sdf>

      <author>
        <name>My Name</name>
        <email>me@my.email</email>
      </author>

      <description>
        My simple mobile manipulator
      </description>
    </model>
    ~~~

1. Next, create the model SDF file:

        gedit ~/.gazebo/models/simple_mobile_manipulator/model.sdf

    and populate with following contents:

    ~~~
    <?xml version="1.0" ?>
    <sdf version="1.3">
      <model name="simple_mobile_manipulator">

        <include>
          <uri>model://my_gripper</uri>
          <pose>1.3 0 0.1 0 0 0</pose>
        </include>

        <include>
          <uri>model://my_robot</uri>
          <pose>0 0 0 0 0 0</pose>
        </include>

        <joint name="arm_gripper_joint" type="revolute">
          <parent>mobile_base::chassis</parent>
          <child>simple_gripper::riser</child>
          <axis>
            <limit>
              <lower>0</lower>
              <upper>0</upper>
            </limit>
            <xyz>0 0 1</xyz>
          </axis>
        </joint>

        <!-- attach sensor to the gripper -->
        <include>
          <uri>model://hokuyo</uri>
          <pose>1.3 0 0.3 0 0 0</pose>
        </include>

        <joint name="hokuyo_joint" type="revolute">
          <child>hokuyo::link</child>
          <parent>simple_gripper::palm</parent>
          <axis>
            <xyz>0 0 1</xyz>
            <limit>
              <upper>0</upper>
              <lower>0</lower>
            </limit>
          </axis>
        </joint>

      </model>
    </sdf>
    ~~~

1. Make sure the `model.config` and `model.sdf` files above are saved, start Gazebo and spawn the model above by using the **select** tab and choosing **Simple Mobile Manipulator** model.  You should see something similar to:

    <img src="http://gazebosim.org/w/images/a/ab/Simple_mobile_manipulator.png" width="640px"/>
