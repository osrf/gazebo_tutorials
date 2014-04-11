#Tutorial: Make a Simple Gripper#

This tutorial describes how to make a simple two-bar pinching gripper.

## Setup your model directory

Reference [Model Database documentation](http://gazebosim.org/user_guide/started__models__database.html) and [SDF](http://gazebosim.org/sdf) documentation for this tutorial.

## Make the model

1. Create a directory for the world file.

        mkdir ~/simple_gripper_tutorial; cd ~/simple_gripper_tutorial


1.  We will begin with a simple empty world.  Create a world file:

        gedit ~/simple_gripper_tutorial/gripper.world

    Copy the following SDF into `gripper.world`:

    ~~~
      <?xml version="1.0"?>
      <sdf version="1.4">
        <world name="default">

        <!-- A ground plane -->
        <include>
          <uri>model://ground_plane</uri>
        </include>

        <!-- A global light source -->
        <include>
          <uri>model://sun</uri>
        </include>

        <include>
          <uri>model://my_gripper</uri>
        </include>

        </world>
      </sdf>
    ~~~

1. Create a model directory inside ~/.gazebo. This is where we'll put the model files:

        mkdir -p ~/.gazebo/models/my_gripper
        cd ~/.gazebo/models/my_gripper

1. Let's layout the basic structure of our gripper. The easiest way to accomplish this is to make a `static` model and add in the links one at a time. A static model means the links will not move when the simulator starts. This will allow you to start the simulator, and visually inspect the link placement before adding joints.

1. Create a `model.config` file

        gedit ~/.gazebo/models/my_gripper/model.config

1. Fill `model.config` with the following contents:

    ~~~~
        <?xml version="1.0"?>

        <model>
          <name>My Gripper</name>
          <version>1.0</version>
          <sdf version='1.4'>simple_gripper.sdf</sdf>

          <author>
            <name>My Name</name>
            <email>me@my.email</email>
          </author>

          <description>
            My awesome robot.
          </description>
        </model>
    ~~~~

1. Make a new file for the model:

        gedit ~/.gazebo/models/my_gripper/simple_gripper.sdf

1. Copy the following code in the `simple_gripper.sdf` file.

    ~~~
    <?xml version="1.0"?>
    <sdf version="1.4">
        <model name="simple_gripper">
            <link name="riser">
                <pose>-0.15 0.0 0.5 0 0 0</pose>
                <inertial>
                    <pose>0 0 -0.5 0 0 0</pose>
                    <inertia>
                        <ixx>0.01</ixx>
                        <ixy>0</ixy>
                        <ixz>0</ixz>
                        <iyy>0.01</iyy>
                        <iyz>0</iyz>
                        <izz>0.01</izz>
                    </inertia>
                    <mass>10.0</mass>
                </inertial>
                <collision name="collision">
                    <geometry>
                        <box>
                            <size>0.2 0.2 1.0</size>
                        </box>
                    </geometry>
                </collision>
                <visual name="visual">
                    <geometry>
                        <box>
                            <size>0.2 0.2 1.0</size>
                        </box>
                    </geometry>
                    <material>
                        <script>Gazebo/Purple</script>
                    </material>
                </visual>
            </link>
            <link name="palm">
                <pose>0.0 0.0 0.05 0 0 0</pose>
                <inertial>
                    <inertia>
                        <ixx>0.01</ixx>
                        <ixy>0</ixy>
                        <ixz>0</ixz>
                        <iyy>0.01</iyy>
                        <iyz>0</iyz>
                        <izz>0.01</izz>
                    </inertia>
                    <mass>0.5</mass>
                </inertial>
                <collision name="collision">
                    <geometry>
                        <box>
                            <size>0.1 0.2 0.1</size>
                        </box>
                    </geometry>
                </collision>
                <visual name="visual">
                    <geometry>
                        <box>
                            <size>0.1 0.2 0.1</size>
                        </box>
                    </geometry>
                    <material>
                        <script>Gazebo/Red</script>
                    </material>
                </visual>
            </link>
            <link name="left_finger">
                <pose>0.1 0.2 0.05 0 0 -0.78539</pose>
                <inertial>
                    <inertia>
                        <ixx>0.01</ixx>
                        <ixy>0</ixy>
                        <ixz>0</ixz>
                        <iyy>0.01</iyy>
                        <iyz>0</iyz>
                        <izz>0.01</izz>
                    </inertia>
                    <mass>0.1</mass>
                </inertial>
                <collision name="collision">
                    <geometry>
                        <box>
                            <size>0.1 0.3 0.1</size>
                        </box>
                    </geometry>
                </collision>
                <visual name="visual">
                    <geometry>
                        <box>
                            <size>0.1 0.3 0.1</size>
                        </box>
                    </geometry>
                    <material>
                        <script>Gazebo/Blue</script>
                    </material>
                </visual>
            </link>
            <link name="left_finger_tip">
                <pose>0.336 0.3 0.05 0 0 1.5707</pose>
                <inertial>
                    <inertia>
                        <ixx>0.01</ixx>
                        <ixy>0</ixy>
                        <ixz>0</ixz>
                        <iyy>0.01</iyy>
                        <iyz>0</iyz>
                        <izz>0.01</izz>
                    </inertia>
                    <mass>0.1</mass>
                </inertial>
                <collision name="collision">
                    <geometry>
                        <box>
                            <size>0.1 0.2 0.1</size>
                        </box>
                    </geometry>
                </collision>
                <visual name="visual">
                    <geometry>
                        <box>
                            <size>0.1 0.2 0.1</size>
                        </box>gedit ~/.gazebo/models/my_gripper/simple_gripper.sdf
                    </geometry>
                    <material>
                        <script>Gazebo/Blue</script>
                    </material>
                </visual>
            </link>
            <link name="right_finger">
                <pose>0.1 -0.2 0.05 0 0 .78539</pose>
                <inertial>
                    <inertia>
                        <ixx>0.01</ixx>
                        <ixy>0</ixy>
                        <ixz>0</ixz>
                        <iyy>0.01</iyy>
                        <iyz>0</iyz>
                        <izz>0.01</izz>
                    </inertia>
                    <mass>0.1</mass>
                </inertial>
                <collision name="collision">
                    <geometry>
                        <box>
                            <size>0.1 0.3 0.1</size>
                        </box>
                    </geometry>
                </collision>
                <visual name="visual">
                    <geometry>
                        <box>
                            <size>0.1 0.3 0.1</size>
                        </box>
                    </geometry>
                    <material>
                        <script>Gazebo/Green</script>
                    </material>
                </visual>
            </link>
            <link name="right_finger_tip">
                <pose>0.336 -0.3 0.05 0 0 1.5707</pose>
                <inertial>
                    <inertia>
                        <ixx>0.01</ixx>
                        <ixy>0</ixy>
                        <ixz>0</ixz>
                        <iyy>0.01</iyy>
                        <iyz>0</iyz>
                        <izz>0.01</izz>
                    </inertia>
                    <mass>0.1</mass>
                </inertial>
                <collision name="collision">
                    <geometry>
                        <box>
                            <size>0.1 0.2 0.1</size>
                        </box>
                    </geometry>
                </collision>
                <visual name="visual">
                    <geometry>
                        <box>
                            <size>0.1 0.2 0.1</size>
                        </box>
                    </geometry>
                    <material>
                        <script>Gazebo/Green</script>
                    </material>
                </visual>
            </link>
            <static>true</static>
        </model>
    </sdf>
    ~~~

1. Run the world file to visualize what we have created up to this point.

        gazebo ~/simple_gripper_tutorial/gripper.world

    You should see something like this:

    <img src="http://gazebosim.org/w/images/6/66/Simple-gripper-1.png" width="640px"/>

1. Once we are happy with the layout of the links, we can add in the joints, by copying the following code into the `simple_gripper.sdf` file before the `</model>` line.

        gedit ~/.gazebo/models/my_gripper/simple_gripper.sdf

    ~~~
            <joint name="palm_left_finger" type="revolute">
                <pose>0 -0.15 0 0 0 0</pose>
                <child>left_finger</child>
                <parent>palm</parent>
                <axis>
                    <limit>
                        <lower>-0.4</lower>
                        <upper>0.4</upper>
                    </limit>
                    <xyz>0 0 1</xyz>
                </axis>
            </joint>
            <joint name="left_finger_tip" type="revolute">
                <pose>0 0.1 0 0 0 0</pose>
                <child>left_finger_tip</child>
                <parent>left_finger</parent>
                <axis>
                    <limit>
                        <lower>-0.4</lower>
                        <upper>0.4</upper>
                    </limit>
                    <xyz>0 0 1</xyz>
                </axis>
            </joint>
            <joint name="palm_right_finger" type="revolute">
                <pose>0 0.15 0 0 0 0</pose>
                <child>right_finger</child>
                <parent>palm</parent>
                <axis>
                    <limit>
                        <lower>-0.4</lower>
                        <upper>0.4</upper>
                    </limit>
                    <xyz>0 0 1</xyz>
                </axis>
            </joint>
            <joint name="right_finger_tip" type="revolute">
                <pose>0 0.1 0 0 0 0</pose>
                <child>right_finger_tip</child>
                <parent>right_finger</parent>
                <axis>
                    <limit>
                        <lower>-0.4</lower>
                        <upper>0.4</upper>
                    </limit>
                    <xyz>0 0 1</xyz>
                </axis>
            </joint>
            <joint name="palm_riser" type="prismatic">
                <child>palm</child>
                <parent>riser</parent>
                <axis>
                    <limit>
                        <lower>0</lower>
                        <upper>0.9</upper>
                    </limit>
                    <xyz>0 0 1</xyz>
                </axis>
            </joint>
    ~~~

    And make the model non-static:

    ~~~
            ...
            <static>false</static>
            ...
    ~~~

1. Start Gazebo again:

        gazebo ~/simple_gripper_tutorial/gripper.world

1. Right-click on the model and select "View->Joints". The newly created joints will be displayed:

    <img src="http://gazebosim.org/w/images/2/2a/Simple-gripper-joints.png" width="640px"/>

1. You can also control the forces on each joint using the Joint Control widget.  Click on the gripper model.  Then expand this widget by clicking on the vertical handle on the right side of the GUI and dragging it to the left. The widget displays a list of sliders, one for each joint. Select the Force tab and use the sliders to apply forces to each joint, and you should see the gripper move.  E.g., set the force on `palm_riser` to 10 (Newtons), and you should see something like:

    <img src="http://gazebosim.org/w/images/a/af/Simple_gripper_joint_control_1_4.png" width="640px"/>

1. Optional:

    1.  Add a small box or cylinder to the world, and position it in the gripper.
    1.  Try to use the joint control GUI interface to pick up the object.

    Tip: You may need to adjust reasonable inertia to the object.

## Next

[Next: Attach Gripper to Robot](http://gazebosim.org/tutorials/?tut=attach_gripper)
