# Tutorial: Make a Mobile Robot#

The tutorial demonstrates Gazebo's basic model management, and exercises familiarity with basic model representation inside the model database by taking the user through the process of creating a two wheeled mobile robot that uses a differential drive mechanism for movement.

## Setup your model directory ##

Read through the [Model Database documentation](http://gazebosim.org/user_guide/started__models__database.html). You will be creating your own model, which must follow the formatting rules for the Gazebo Model Database directory structure.  Also, for details on model description formats, please refer to the [SDF reference](http://gazebosim.org/sdf).

1.  Create a model directory:

        mkdir -p ~/.gazebo/models/my_robot


1.  Create a model config file:

        gedit ~/.gazebo/models/my_robot/model.config

1.  Paste in the following contents:

    ~~~
        <?xml version="1.0"?>
        <model>
          <name>My Robot</name>
          <version>1.0</version>
          <sdf version='1.4'>model.sdf</sdf>

          <author>
           <name>My Name</name>
           <email>me@my.email</email>
          </author>

          <description>
            My awesome robot.
          </description>
        </model>
    ~~~

1.  Create a `~/.gazebo/models/my_robot/model.sdf` file.

        gedit ~/.gazebo/models/my_robot/model.sdf


1. Paste in the following.

    ~~~
        <?xml version='1.0'?>
        <sdf version='1.4'>
          <model name="my_robot">
          </model>
        </sdf>
    ~~~

At this point we have the basic contents for a model. The `model.config` file describes the robot with some extra meta data. The `model.sdf` file contains the necessary tags to instantiate a model named `my_robot` using Gazebo linked against SDF version 1.4.

## Build the Model's Structure ##

This step will create a rectangular base with two wheels.

It is important to start simple, and build up a model in steps. The first step is to layout the basic shapes of the model. To do this we will make our model `static`, which means it will be ignored by the physics engine. As a result the model will stay in one place and allow us to properly align all the components.

1.  Make the model static by adding a `<static>true</static>` element to the `~/.gazebo/models/my_robot/model.sdf` file:

%%%
<?xml version='1.0'?>
<sdf version='1.4'>
  <model name="myrobot">
%%%

    ~~~
            <static>true</static>
    ~~~

    %%%
          </model>
        </sdf>
    %%%

1.  Add the rectangular base by editing the `~/.gazebo/models/my_robot/model.sdf` file:

    %%%
        <?xml version='1.0'?>
          <sdf version='1.4'>
            <model name="my_robot">
            <static>true</static>
    %%%
    ~~~
              <link name='chassis'>
                <pose>0 0 .1 0 0 0</pose>

                <collision name='collision'>
                  <geometry>
                    <box>
                      <size>.4 .2 .1</size>
                    </box>
                  </geometry>
                </collision>

                <visual name='visual'>
                  <geometry>
                    <box>
                      <size>.4 .2 .1</size>
                    </box>
                  </geometry>
                </visual>
              </link>
    ~~~
    %%%
          </model>
        </sdf>
    %%%

  Here we have created a `box` with a size of `0.4 x 0.2 x 0.1` meters. The `collision` element specifies the shape used by the collision detection engine. The `visual` element specifies the shape used by the rendering engine. For most use cases the `collision` and `visual` elements are the same. The most common use for different `collision` and `visual` elements is to have a simplified `collision` element paired with a `visual` element that uses a complex mesh. This will help improve performance.

1.  Try out your model by running gazebo, and importing your model through the [[insert_model_gui | Insert Model]] interface on the GUI.

        gazebo

  You should see a white box floating .1 meters above the ground plane.

  <img src="http://gazebosim.org/w/images/8/8c/My_robot_box.png" width="640px"/>

1.  Now we can add a caster to the robot. The caster is a sphere with no friction. This kind of caster is better than adding a wheel with a joint since it places fewer constraints on the physics engine.

    %%%
        <?xml version='1.0'?>
        <sdf version='1.4'>
          <model name="my_robot">
            <static>true</static>
            <link name='chassis'>
              <pose>0 0 .1 0 0 0</pose>
              <collision name='collision'>
                <geometry>
                  <box>
                    <size>.4 .2 .1</size>
                  </box>
                </geometry>
              </collision>

              <visual name='visual'>
                <geometry>
                  <box>
                    <size>.4 .2 .1</size>
                  </box>
                </geometry>
              </visual>
    %%%

    ~~~
              <collision name='caster_collision'>
                <pose>-0.15 0 -0.05 0 0 0</pose>
                <geometry>
                    <sphere>
                    <radius>.05</radius>
                  </sphere>
                </geometry>

                <surface>
                  <friction>
                    <ode>
                      <mu>0</mu>
                      <mu2>0</mu2>
                      <slip1>1.0</slip1>
                      <slip2>1.0</slip2>
                    </ode>
                  </friction>
                </surface>
              </collision>

              <visual name='caster_visual'>
                <pose>-0.15 0 -0.05 0 0 0</pose>
                <geometry>
                  <sphere>
                    <radius>.05</radius>
                  </sphere>
                </geometry>
              </visual>
    ~~~

    %%%
            </link>
          </model>
        </sdf>
    %%%

  Try out your model to make sure the caster appears at the end of the robot.  Spawn it in gazebo to see (you don't need to restart Gazebo; it will reload your modified model from disk each time you insert it):

  <img src="http://gazebosim.org/w/images/f/f1/My_robot_caster.png" width="640px"/>

1.  Now let's add a left wheel. Modify the `~/.gazebo/models/my_robot/model.sdf` file to be the following:

    %%%
        <?xml version='1.0'?>
        <sdf version='1.4'>
          <model name="my_robot">
            <static>true</static>
            <link name='chassis'>
              <pose>0 0 .1 0 0 0</pose>
              <collision name='collision'>
                <geometry>
                  <box>
                    <size>.4 .2 .1</size>
                  </box>
                </geometry>
              </collision>

              <visual name='visual'>
                <geometry>
                  <box>
                    <size>.4 .2 .1</size>
                  </box>
                </geometry>
              </visual>

              <collision name='caster_collision'>
                <pose>-0.15 0 -0.05 0 0 0</pose>
                <geometry>
                  <sphere>
                  <radius>.05</radius>
                </sphere>
              </geometry>

              <surface>
                <friction>
                  <ode>
                    <mu>0</mu>
                    <mu2>0</mu2>
                    <slip1>1.0</slip1>
                    <slip2>1.0</slip2>
                  </ode>
                </friction>
              </surface>
            </collision>

            <visual name='caster_visual'>
              <pose>-0.15 0 -0.05 0 0 0</pose>
              <geometry>
                <sphere>
                  <radius>.05</radius>
                </sphere>
              </geometry>
            </visual>
          </link>
    %%%
    ~~~
          <link name="left_wheel">
            <pose>0.1 0.13 0.1 0 1.5707 1.5707</pose>
            <collision name="collision">
              <geometry>
                <cylinder>
                  <radius>.1</radius>
                  <length>.05</length>
                </cylinder>
              </geometry>
            </collision>
            <visual name="visual">
              <geometry>
                <cylinder>
                  <radius>.1</radius>
                  <length>.05</length>
                </cylinder>
              </geometry>
            </visual>
          </link>
    ~~~
    %%%
          </model>
        </sdf>
    %%%

  Run Gazebo, insert your robot model and make sure the wheel has appeared and is in the correct location.

  <img src="http://gazebosim.org/w/images/6/67/My_robot_caster_left_wheel.png" width="640px"/>


1.  We can make a right wheel by copying the left wheel, and adjusting the wheel link's pose:

    %%%
        <?xml version='1.0'?>
        <sdf version='1.4'>
          <model name="my_robot">
            <static>true</static>
            <link name='chassis'>
              <pose>0 0 .1 0 0 0</pose>
              <collision name='collision'>
                <geometry>
                  <box>
                    <size>.4 .2 .1</size>
                  </box>
                </geometry>
              </collision>

              <visual name='visual'>
                <geometry>
                  <box>
                    <size>.4 .2 .1</size>
                  </box>
                </geometry>
              </visual>

              <collision name='caster_collision'>
                <pose>-0.15 0 -0.05 0 0 0</pose>
                <geometry>
                  <sphere>
                  <radius>.05</radius>
                </sphere>
              </geometry>

              <surface>
                <friction>
                  <ode>
                    <mu>0</mu>
                    <mu2>0</mu2>
                    <slip1>1.0</slip1>
                    <slip2>1.0</slip2>
                  </ode>
                </friction>
              </surface>
            </collision>

            <visual name='caster_visual'>
              <pose>-0.15 0 -0.05 0 0 0</pose>
              <geometry>
                <sphere>
                  <radius>.05</radius>
                </sphere>
              </geometry>
            </visual>
          </link>
          <link name="left_wheel">
            <pose>0.1 0.13 0.1 0 1.5707 1.5707</pose>
            <collision name="collision">
              <geometry>
                <cylinder>
                  <radius>.1</radius>
                  <length>.05</length>
                </cylinder>
              </geometry>
            </collision>
            <visual name="visual">
              <geometry>
                <cylinder>
                  <radius>.1</radius>
                  <length>.05</length>
                </cylinder>
              </geometry>
            </visual>
          </link>
    %%%
    ~~~
          <link name="right_wheel">
            <pose>0.1 -0.13 0.1 0 1.5707 1.5707</pose>
            <collision name="collision">
              <geometry>
                <cylinder>
                  <radius>.1</radius>
                  <length>.05</length>
                </cylinder>
              </geometry>
            </collision>
            <visual name="visual">
              <geometry>
                <cylinder>
                  <radius>.1</radius>
                  <length>.05</length>
                </cylinder>
              </geometry>
            </visual>
          </link>
    ~~~
    %%%
          </model>
        </sdf>
    %%%

  At this point the robot should have a chassis with a caster and two wheels.

  <img src="http://gazebosim.org/w/images/1/16/My_robot_caster_wheels.png" width="640px"/>

1. Make the model dynamic by setting `<static>` to false, and add two hinge joints for the left and right wheels.

    %%%
        <?xml version='1.0'?>
        <sdf version='1.4'>
          <model name="my_robot">
    %%%
    ~~~
            <static>false</static>
    ~~~
    %%%
            <link name='chassis'>
              <pose>0 0 .1 0 0 0</pose>
              <collision name='collision'>
                <geometry>
                  <box>
                    <size>.4 .2 .1</size>
                  </box>
                </geometry>
              </collision>

              <visual name='visual'>
                <geometry>
                  <box>
                    <size>.4 .2 .1</size>
                  </box>
                </geometry>
              </visual>

              <collision name='caster_collision'>
                <pose>-0.15 0 -0.05 0 0 0</pose>
                <geometry>
                  <sphere>
                  <radius>.05</radius>
                </sphere>
              </geometry>

              <surface>
                <friction>
                  <ode>
                    <mu>0</mu>
                    <mu2>0</mu2>
                    <slip1>1.0</slip1>
                    <slip2>1.0</slip2>
                  </ode>
                </friction>
              </surface>
            </collision>

            <visual name='caster_visual'>
              <pose>-0.15 0 -0.05 0 0 0</pose>
              <geometry>
                <sphere>
                  <radius>.05</radius>
                </sphere>
              </geometry>
            </visual>
          </link>
          <link name="left_wheel">
            <pose>0.1 0.13 0.1 0 1.5707 1.5707</pose>
            <collision name="collision">
              <geometry>
                <cylinder>
                  <radius>.1</radius>
                  <length>.05</length>
                </cylinder>
              </geometry>
            </collision>
            <visual name="visual">
              <geometry>
                <cylinder>
                  <radius>.1</radius>
                  <length>.05</length>
                </cylinder>
              </geometry>
            </visual>
          </link>

          <link name="right_wheel">
            <pose>0.1 -0.13 0.1 0 1.5707 1.5707</pose>
            <collision name="collision">
              <geometry>
                <cylinder>
                  <radius>.1</radius>
                  <length>.05</length>
                </cylinder>
              </geometry>
            </collision>
            <visual name="visual">
              <geometry>
                <cylinder>
                  <radius>.1</radius>
                  <length>.05</length>
                </cylinder>
              </geometry>
            </visual>
          </link>
    %%%
    ~~~
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
    ~~~
    %%%
          </model>
        </sdf>
    %%%

    The two joints rotate about the y axis `<xyz>0 1 0</xyz>`, and connect each wheel to the chassis.

1. Start gazebo, and insert your model. Click on the three white rectangles to the right of the screen and drag them to the left.

1. A new window should appear that contains various controllers for each joint. ('''Note''' Make sure the model you want to control is selected)

1. Under the `Force` tab, increase the force applied to each joint to about 0.1N-m. The robot should move around:

  <img src="http://gazebosim.org/w/images/4/48/Simple-robot-driving.png" width="640px"/>

1. Congrats, you now have a basic mobile robot.


## Try for yourself ##

1.  Be creative and make a new robot.

    Idea: A quadruped that consists of torso with four cylindrical legs. Each leg is attached to the torso with a revolute joint.

    Idea: A six wheeled vehicle with a scoop front loading mechanism.

## Next ##
[Next: Attach Meshes](http://gazebosim.org/tutorials/?tut=attach_meshes)
