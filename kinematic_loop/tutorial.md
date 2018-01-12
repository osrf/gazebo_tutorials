# Introduction
This tutorial shows how to model a 4-bar linkage in Gazebo,
with examples using SDFormat and URDF.
This is a challenge because
[URDF uses a tree structure](http://wiki.ros.org/urdf/Tutorials/Create%20your%20own%20urdf%20file)
to represent robot kinematics,
which does not permit closed loops like a 4-bar linkage.
SDFormat is able to model closed kinematic loops
because it uses a graph structure.
Since gazebo converts URDF to SDF before loading,
we can declare an SDFormat joint inside `<gazebo>`
extension tags that will get added to the model 
after being converted to SDFormat.


# Simple 4-bar linkage in SDFormat

This is an example of a 4-bar linkage connected to the ground at each end
expressed in SDFormat.
It has 3 links and 4 revolute joints.

![screenshot of four\_bar_sdf model](https://bytebucket.org/osrf/gazebo_tutorials/raw/kinematic_loop/kinematic_loop/four_bar.png)

The model files are in the [four\_bar_sdf](https://bitbucket.org/osrf/gazebo_tutorials/src/kinematic_loop/kinematic_loop/four_bar_sdf)
folder.
To use this model, create a folder `~/.gazebo/models/four_bar_sdf` and copy the
[model.config](https://bitbucket.org/osrf/gazebo_tutorials/raw/kinematic_loop/kinematic_loop/four_bar_sdf/model.config)
and
[model.sdf](https://bitbucket.org/osrf/gazebo_tutorials/raw/kinematic_loop/kinematic_loop/four_bar_sdf/model.sdf)
to that folder.
You can also examine the
[model template](https://bitbucket.org/osrf/gazebo_tutorials/src/kinematic_loop/kinematic_loop/four_bar_sdf/model.sdf.erb)
created with embedded ruby to see how the model is constructed.

# Split 4-bar linkage with an extra fixed joint

A model with kinematic loops can be partially modeled as a tree by
cutting some of the loops.
This can be done by removing a joint or by splitting a link
in half.
The loop can be closed again by adding an extra joint.
In this example, the middle link is split in half
and a fixed joint is added to hold them together.
It has the same 4 revolute joints but with 4 links and a fixed joint.

![screenshot of four\_bar\_split\_fixed_sdf model](https://bytebucket.org/osrf/gazebo_tutorials/raw/kinematic_loop/kinematic_loop/four_bar_split.png)

The SDFormat model files are in the [four\_bar\_split\_fixed_sdf](https://bitbucket.org/osrf/gazebo_tutorials/src/kinematic_loop/kinematic_loop/four_bar_split_fixed_sdf)
folder.
To use this model, create a folder `~/.gazebo/models/four_bar_split_fixed_sdf` and copy the
[model.config](https://bitbucket.org/osrf/gazebo_tutorials/raw/kinematic_loop/kinematic_loop/four_bar_split_fixed_sdf/model.config)
and
[model.sdf](https://bitbucket.org/osrf/gazebo_tutorials/raw/kinematic_loop/kinematic_loop/four_bar_split_fixed_sdf/model.sdf)
to that folder.
You can also examine the
[model template](https://bitbucket.org/osrf/gazebo_tutorials/src/kinematic_loop/kinematic_loop/four_bar_split_fixed_sdf/model.sdf.erb)
created with embedded ruby to see how the model is constructed.

# Split 4-bar linkage in URDF with an SDFormat fixed joint

The split 4-bar linkage is modeled in URDF in the
[four\_bar\_split\_fixed_urdf](https://bitbucket.org/osrf/gazebo_tutorials/src/kinematic_loop/kinematic_loop/four_bar_split_fixed_urdf)
folder with
[model.config](https://bitbucket.org/osrf/gazebo_tutorials/raw/kinematic_loop/kinematic_loop/four_bar_split_fixed_urdf/model.config)
and
[model.urdf](https://bitbucket.org/osrf/gazebo_tutorials/raw/kinematic_loop/kinematic_loop/four_bar_split_fixed_urdf/model.urdf)
files and the embedded ruby
[model template](https://bitbucket.org/osrf/gazebo_tutorials/src/kinematic_loop/kinematic_loop/four_bar_split_fixed_urdf/model.urdf.erb).
The SDFormat fixed joint is
[specified in \<gazebo\> tags](https://bitbucket.org/osrf/gazebo_tutorials/src/kinematic_loop/kinematic_loop/four_bar_split_fixed_urdf/model.urdf?at=kinematic_loop&fileviewer=file-view-default#model.urdf-149:157):

<include lang='xml' from='/  .!-- SDFormat/' src='https://bitbucket.org/osrf/gazebo_tutorials/raw/kinematic_loop/kinematic_loop/four_bar_split_fixed_urdf/model.urdf' />


