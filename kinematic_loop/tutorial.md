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
extension tags to close the loop.
That joint will get added to the model after being converted to SDFormat.


# Simple 4-bar linkage in SDFormat

This is an SDFormat example of a 4-bar linkage connected to the ground at each end.
It has 4 revolute joints labeled joint\_A, joint\_B, joint\_C, and joint\_D
and 3 links named link\_AB, link\_BC, and link\_CD.

![screenshot of four\_bar_sdf model](https://github.com/osrf/gazebo_tutorials/raw/master/kinematic_loop/four_bar.png)

The model files are in the [four\_bar_sdf](https://github.com/osrf/gazebo_tutorials/blob/master/kinematic_loop/four_bar_sdf)
folder.
To use this model, create a folder `~/.gazebo/models/four_bar_sdf` and copy the
[model.config](https://github.com/osrf/gazebo_tutorials/raw/master/kinematic_loop/four_bar_sdf/model.config)
and
[model.sdf](https://github.com/osrf/gazebo_tutorials/raw/master/kinematic_loop/four_bar_sdf/model.sdf)
to that folder.
You can then insert the model into a simulation using the `Insert` panel
in the [left side of the gazebo client](http://gazebosim.org/tutorials?cat=guided_b&tut=guided_b2).

For brevity, the model parameters are encoded in an embedded ruby template file named
[model.sdf.erb](https://github.com/osrf/gazebo_tutorials/raw/master/kinematic_loop/four_bar_sdf/model.sdf.erb).
The model.sdf can be generated from the template using the `erb` command: `erb -T 1 model.sdf.erb > model.sdf`.

This allows geometric parameters to be defined in one place
along with a helper function for computing the moment of inertia of a uniform box.

<include lang='ruby' from='/  # Box dimensions/' to='/  # Points/' src='https://github.com/osrf/gazebo_tutorials/raw/master/kinematic_loop/four_bar_sdf/model.sdf.erb' />

The link parameters are stored in a dictionary named `boxes`:

<include lang='ruby' from='/  # Points/' to='/  # Revolute/' src='https://github.com/osrf/gazebo_tutorials/raw/master/kinematic_loop/four_bar_sdf/model.sdf.erb' />

and the joint parameters are stored in a dictionary named `joints`:

<include lang='ruby' from='/  # Revolute/' to='/  # end first ruby code block/' src='https://github.com/osrf/gazebo_tutorials/raw/master/kinematic_loop/four_bar_sdf/model.sdf.erb' />

A model template is then included that references the computed parameters
for each link and joint:

<include lang='xml' from='/.sdf version/' to='@/sdf.@' src='https://github.com/osrf/gazebo_tutorials/raw/master/kinematic_loop/four_bar_sdf/model.sdf.erb' />

The full [model.sdf](https://github.com/osrf/gazebo_tutorials/raw/master/kinematic_loop/four_bar_sdf/model.sdf)
is instantiated from this template, as you can see in the snippet below:

<include lang='xml' from='/    .link name="link_CD"./' to='/joint_B. type=.revolute../' src='https://github.com/osrf/gazebo_tutorials/raw/master/kinematic_loop/four_bar_sdf/model.sdf' />


# Split 4-bar linkage with an extra fixed joint

A model with kinematic loops can be partially modeled as a tree by
cutting some of the loops.
This can be done by removing a joint or by splitting a link
in half.
The loop can be closed again by adding an extra joint.
In this example, the middle link is split in half
and a fixed joint is added to hold them together.
It has the same 4 revolute joints joint\_A, joint\_B, joint\_C, and joint\_D
but with 4 links link\_AB, link\_BE, link\_EC, link\_CD
and a fixed joint joint\_E.

![screenshot of four\_bar\_split\_fixed_sdf model](https://github.com/osrf/gazebo_tutorials/raw/master/kinematic_loop/four_bar_split.png)

The SDFormat model files are in the [four\_bar\_split\_fixed_sdf](https://github.com/osrf/gazebo_tutorials/blob/master/kinematic_loop/four_bar_split_fixed_sdf)
folder.
To use this model, create a folder `~/.gazebo/models/four_bar_split_fixed_sdf` and copy the
[model.config](https://github.com/osrf/gazebo_tutorials/raw/master/kinematic_loop/four_bar_split_fixed_sdf/model.config)
and
[model.sdf](https://github.com/osrf/gazebo_tutorials/raw/master/kinematic_loop/four_bar_split_fixed_sdf/model.sdf)
to that folder.
You can also examine the
[model template](https://github.com/osrf/gazebo_tutorials/raw/master/kinematic_loop/four_bar_split_fixed_sdf/model.sdf.erb)
created with embedded ruby to see how the model is constructed.

The difference between the `four_bar_sdf` and `four_bar_split_fixed_sdf`
model templates is shown below:

~~~
--- a/kinematic_loop/four_bar_sdf/model.sdf.erb
+++ b/kinematic_loop/four_bar_split_fixed_sdf/model.sdf.erb
@@ -35,15 +35,21 @@
   # ^Y         |          |
   # |          |          |
   # |   X      |          |
-  # O--->      B----------C
+  # O--->      B-----E----C
   #
   boxes = {}

   boxes["AB"] = box_inertia(t, Y, t);
   boxes["AB"][:offset] = Vector[-t/2, -Y/2, 0]

-  boxes["BC"] = box_inertia(X, t, t);
-  boxes["BC"][:offset] = Vector[X/2, -(Y+t/2), 0]
+  # boxes["BC"] = box_inertia(X, t, t);
+  # boxes["BC"][:offset] = Vector[X/2, -(Y+t/2), 0]
+
+  boxes["BE"] = box_inertia(X/2, t, t);
+  boxes["BE"][:offset] = Vector[X/4, -(Y+t/2), 0]
+
+  boxes["EC"] = box_inertia(X/2, t, t);
+  boxes["EC"][:offset] = Vector[X*3/4, -(Y+t/2), 0]

   boxes["CD"] = box_inertia(t, Y, t);
   boxes["CD"][:offset] = Vector[X+t/2, -Y/2, 0]
@@ -61,8 +67,8 @@
   joints["B"] = {}
   joints["B"][:type] = "revolute"
   joints["B"][:parent] = "link_AB"
-  joints["B"][:child] = "link_BC"
-  joints["B"][:pose] = Vector[-X/2, t/2, 0, 0, 0, 0]
+  joints["B"][:child] = "link_BE"
+  joints["B"][:pose] = Vector[-X/4, t/2, 0, 0, 0, 0]
   joints["B"][:axis] = Vector[0, 0, 1]

   joints["D"] = {}
@@ -75,11 +81,15 @@
   joints["C"] = {}
   joints["C"][:type] = "revolute"
   joints["C"][:parent] = "link_CD"
-  joints["C"][:child] = "link_BC"
-  joints["C"][:pose] = Vector[X/2, t/2, 0, 0, 0, 0]
+  joints["C"][:child] = "link_EC"
+  joints["C"][:pose] = Vector[X/4, t/2, 0, 0, 0, 0]
   joints["C"][:axis] = Vector[0, 0, 1]

-  # end first ruby code block
+  joints["E"] = {}
+  joints["E"][:type] = "fixed"
+  joints["E"][:parent] = "link_BE"
+  joints["E"][:child] = "link_EC"
+  joints["E"][:pose] = Vector[-X/4, 0, 0, 0, 0, 0]
 %>
 <sdf version="1.5">
   <model name="four_bar_sdf">
@@ -134,9 +144,15 @@
       <pose><%= a_to_s(joint[:pose]) %></pose>
       <parent><%= joint[:parent] %></parent>
       <child><%= joint[:child] %></child>
+<%
+    if joint.has_key?(:axis)
+%>
       <axis>
         <xyz><%= a_to_s(joint[:axis]) %></xyz>
       </axis>
+<%
+    end
+%>
     </joint>
 <%
   end
~~~

# Split 4-bar linkage in URDF with an SDFormat fixed joint

The revolute joints in the split 4-bar linkage can be modeled in URDF
with the fixed joint added using a `<gazebo>` extension tag.
This URDF model is in the
[four\_bar\_split\_fixed_urdf](https://github.com/osrf/gazebo_tutorials/blob/master/kinematic_loop/four_bar_split_fixed_urdf)
folder with
[model.config](https://github.com/osrf/gazebo_tutorials/raw/master/kinematic_loop/four_bar_split_fixed_urdf/model.config)
and
[model.urdf](https://github.com/osrf/gazebo_tutorials/raw/master/kinematic_loop/four_bar_split_fixed_urdf/model.urdf)
files and the embedded ruby
[model template](https://github.com/osrf/gazebo_tutorials/raw/master/kinematic_loop/four_bar_split_fixed_urdf/model.urdf.erb).
The model.urdf can be generated from the template using the erb command: `erb -T 1 model.urdf.erb > model.urdf`.
Comparing this model with the
[four\_bar\_split\_fixed_sdf](https://github.com/osrf/gazebo_tutorials/blob/master/kinematic_loop/four_bar_split_fixed_sdf)
model provides a useful comparison of how coordinate frames are defined in SDFormat and URDF.

The SDFormat fixed joint is
[specified in gazebo tags](https://github.com/osrf/gazebo_tutorials/raw/master/kinematic_loop/four_bar_split_fixed_urdf/model.urdf?at=kinematic_loop&fileviewer=file-view-default#model.urdf-149:158):

<include lang='xml' from='/  .!-- SDFormat/' src='https://github.com/osrf/gazebo_tutorials/raw/master/kinematic_loop/four_bar_split_fixed_urdf/model.urdf' />


