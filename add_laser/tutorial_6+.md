# Overview

**Prerequisites:** [Attach a Mesh as Visual](/tutorials/?tut=attach_meshes)

This tutorials demonstrates how the user can create composite models directly
from other models in the
[Gazebo Model Database](https://github.com/osrf/gazebo_models)
by using the
[\<include\>](http://sdformat.org/spec?ver=1.5&elem=world#world_include)
tags and
[\<joint\>](http://sdformat.org/spec?ver=1.5&elem=joint)
to connect different components of a composite model.

## Adding a Laser

Adding a laser to a robot, or any model, is simply a matter of including the sensor in the model.

1.  Go into your model directory from the previous tutorial:

        cd ~/.gazebo/models/my_robot

1.  Open `model.sdf` in your favorite editor.

1.  Add the following lines directly before the `</model>` tag near the end of the file.

    ~~~
        <include>
          <uri>model://hokuyo</uri>
          <pose>0.2 0 0.2 0 0 0</pose>
        </include>
        <joint name="hokuyo_joint" type="fixed">
          <child>hokuyo::link</child>
          <parent>chassis</parent>
        </joint>
    ~~~

    The `<include>` block tells Gazebo to find a model, and insert it at a
    given `<pose>` relative to the parent model. In this case we place the
    hokuyo laser forward and above the robot.  The `<uri>` block tells gazebo
    where to find the model inside its model database (note, you can see a
    listing of the model database uri used by these tutorials
    [here](http://models.gazebosim.org/), and at the corresponding [git
    repository](https://github.com/osrf/gazebo_models)).

    The new `<joint>` connects the inserted hokuyo laser onto the chassis of the robot. The joint is `fixed` to prevent it from moving.

    The `<child>` name in the joint is derived from the [hokuyo model's SDF](https://github.com/osrf/gazebo_models/blob/master/hokuyo/model.sdf), which begins with:

    %%%
        <?xml version="1.0" ?>
        <sdf version="1.4">
          <model name="hokuyo">
            <link name="link">
    %%%

    When the hokuyo model is inserted, the hokuyo's links are namespaced with their model name. In this case the model name is `hokuyo`, so each link in the hokuyo model is prefaced with `hokuyo::`.

1.  Now start gazebo, and add the robot to the simulation using the Insert tab on the GUI. If the hokuyo model does not exist locally, Gazebo will pull the model from the [Model Database](https://github.com/osrf/gazebo_models). Alternatively, manually download the model files to your local cache:

        cd ~/.gazebo/models
        wget -q -R *index.html*,*.tar.gz --no-parent -r -x -nH http://models.gazebosim.org/hokuyo/

    Once gazebo is launched, you should see the robot with a laser attached.

    [[file:files/Add_laser_pioneer.png]]

1.  (Optional)  Try adding a camera to the robot. The camera's model URI is `model://camera`. For reference, the SDF documentation can be found [here](http://gazebosim.org/sdf/).

## Next

[Next: Make a Simple Gripper](/tutorials/?tut=simple_gripper)
