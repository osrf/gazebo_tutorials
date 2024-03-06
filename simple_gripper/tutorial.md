# Overview

This tutorial describes how to make a simple two-bar pinching gripper by
editing SDF files.

For editing models graphically, see the
[Model Editor](/tutorials?tut=model_editor&cat=build_robot)
tutorial.

# Setup your model directory

Reference [Model Database documentation](/tutorials?tut=model_structure&cat=build_robot) and [SDF](http://gazebosim.org/sdf) documentation for this tutorial.

# Make the model

1. Create a directory for the world file.

        mkdir ~/simple_gripper_tutorial; cd ~/simple_gripper_tutorial


1.  We will begin with a simple empty world.  Create a world file:

        gedit ~/simple_gripper_tutorial/gripper.world

    Copy the following SDF into [gripper.world](http://github.com/osrf/gazebo_tutorials/raw/master/simple_gripper/files/gripper.world):

    <include src='http://github.com/osrf/gazebo_tutorials/raw/master/simple_gripper/files/gripper.world' />


1. Create a model directory inside ~/.gazebo. This is where we'll put the model files:

        mkdir -p ~/.gazebo/models/my_gripper
        cd ~/.gazebo/models/my_gripper

1. Let's layout the basic structure of our gripper. The easiest way to accomplish this is to make a `static` model and add in the links one at a time. A static model means the links will not move when the simulator starts. This will allow you to start the simulator, and visually inspect the link placement before adding joints.

1. Create a [model.config](http://github.com/osrf/gazebo_tutorials/raw/master/simple_gripper/files/model.config) file:

        gedit model.config

1. And copy the following contents:

    <include src='http://github.com/osrf/gazebo_tutorials/raw/master/simple_gripper/files/model.config' />

1. Likewise, create a
[simple_gripper.sdf](http://github.com/osrf/gazebo_tutorials/raw/master/simple_gripper/files/simple_gripper.sdf)
file:

        gedit simple_gripper.sdf

1. And copy the following code into it:

    <include src='http://github.com/osrf/gazebo_tutorials/raw/master/simple_gripper/files/simple_gripper.sdf' />

1. Run the world file to visualize what we have created up to this point.

        gazebo ~/simple_gripper_tutorial/gripper.world

    You should see something like this:

    [[file:files/Simple-gripper-1.png|640px]]

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

1. Right-click on the model and select "View->Joints" and "View->Wireframe". The newly created joints will be displayed:

    [[file:files/Simple-gripper-joints_5.png|640px]]

1. You can also control the forces on each joint using the Joint Control widget.  Click on the gripper model.  Then expand this widget by clicking on the vertical handle on the right side of the GUI and dragging it to the left. The widget displays a list of sliders, one for each joint. Select the Force tab and use the sliders to apply forces to each joint, and you should see the gripper move.  E.g., set the force on `palm_riser` to 10 (Newtons), and you should see something like:

    [[file:files/Simple_gripper_joint_control_1_5.png|640px]]

1. Optional:

    1.  Add a small box or cylinder to the world, and position it in the gripper.
    1.  Try to use the joint control GUI interface to pick up the object.

    Tip: You may need to adjust reasonable inertia to the object.

# Next

[Next: Attach Gripper to Robot](/tutorials/?tut=attach_gripper)
