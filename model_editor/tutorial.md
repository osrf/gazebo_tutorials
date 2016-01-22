# Overview

This tutorial describes the process of creating a model using the Model Editor.

## Open the Model Editor

1.  Make sure Gazebo is [installed](http://gazebosim.org/install).

1.  Start up gazebo.

    ~~~
    $ gazebo
    ~~~

1. On the `Edit` menu, go to `Model Editor`, or hit `Ctrl+M` to open the editor.

    [[file:files/empty_editor_7.png|800px]]

## Graphical user interface

The editor is composed of the following 2 areas:

* The **Palette** on the left has two tabs. The `Insert` tab lets you insert
parts (links and other models) into the scene to build the model. The `Model`
tab displays a list of all the parts that make up the model you are building.

* The **3D View** on the right where you can see a preview of your model and
interact with it to edit its properties and create joints between links.

The GUI tools on the top toolbar can be used to manipulate joints and links in
the 3D View.

# Add Links

## Add simple shapes

The model editor has three simple primitive geometries that the user can insert
into the 3D view to make a link of the model.

1. On the Palette, click on the `box`, `sphere`, or `cylinder` icon under
**Simple Shapes**.

1. Move your mouse cursor over the 3D view to see the visual appear, and
click/release anywhere to add it to the model.

    > **Tip:** You can press `Esc` to cancel adding the current
    link attached to the mouse cursor.

    [[file:files/model_editor_simple_shapes_7.png|640]]

## Add meshes

To add a custom mesh,

1. Click on the `Add` button under **Custom Shapes**, which pops up a dialog
that lets you find the mesh you want to add.

1. Click on `Browse` button and use the file browser to find the mesh file
on your local machine. If you know the path of the mesh file, you can enter it
directly in the text field box next to the `Browse` button. Note Gazebo
currently only supports importing COLLADA (dae) and STereoLithography (stl) files.

1. Click `Import` to load the mesh file. Then, add it to the 3D view.

    [[file:files/model_editor_insert_mesh_7.png|640]]

# Create Joints

The model editor supports creating several types of joints between links in the
model being edited. To create a joint:

1. Click on the `joint` icon on the tool bar. This brings up the Joint Creation
Dialog which allows you to specify different properties of the joint you
want to create. As you can see in the dialog, the default joint type is
a `Revolute` joint.

1. Begin by moving your mouse over the link you wish to create a joint for to
see it being highlighted and click on it. This link will be the parent link of
the joint.

1. Next, move your mouse to the link which you would like to be the child link
of the joint. Click on it to see a colored line connecting the two links and
a joint visual attached to the child link.

    [[file:files/model_editor_joint_7.png|800px]]

    The line representing the joint is color-coded. Play around with different
joint types to see the colors.

    The joint visual consists of RGB axes which help to give an idea of the
coordinate frame of the joint. The yellow arrow indicates the primary axis of
the joint. For example, in the case of a revolute joint, this is the axis of
rotation.

1. Once you have specified all the desired properties of the joint in the
Joint Creation Dialog, click on the `Create` button at the bottom to finalize
joint creation.

    > **Tip:** You can press `Esc` any time to cancel the joint creation process.

# Edit your model

**Note: Be careful when editing your model; the editor currently has no option to undo your actions.**

> **Tip:** All measurements are in meters.

## Edit links

The model editor supports editing properties of a link which you would
also find in its SDF.

> **Note:**  Gazebo 6\+ supports editing
links, visuals, and collisions. The ability to edit sensors and
plugins are to be implemented in later versions.

To edit a link's properties: Double-click on the link or right click and select
`Open Link Inspector`. A dialog window will appear which contains
`Link`, `Visual`, and `Collision` property tabs.

As an example, try changing the link pose and visual colors. Once you are done, click on
`OK` to close the inspector.

[[file:files/model_editor_inspector_7.png|640px]]

## Edit joints

As mentioned earlier, joint properties can also be edited. These are properties
that you would find in the joint SDF.

To edit a joint: Double-click on the line connecting the links or right click
on it and select `Open Joint Inspector`. The joint inspector will appear.

As an example, try changing the joint pose and joint type. Once you are done, click on
`OK` to close the inspector.

[[file:files/model_editor_joint_inspector_7.png|300px]]

# Saving your model

Saving will create a [directory, SDF and config files](http://gazebosim.org/tutorials?tut=model_structure&cat=build_robot) for your model.

As an exercise, let's build a simple car and save it. The car will have a
box chassis and four cylinder wheels. Each wheel will be connected to the
chassis with a revolute joint:

[[file:files/model_editor_car.png|800px]]

Once you're happy with the model you created, go to the `Model` tab in the left
panel and give it a name.

[[file:files/model_editor_edit_name.png|200px]]

To save the model, choose `File`, then `Save As` (or hit `Ctrl+S`) in the top
menu. A dialog will come up where you can choose the location for your model.

[[file:files/model_editor_save_dialog.png|300px]]

# Exit

When you're done creating the model and you've saved it, go to `File` and then
`Exit Model Editor`.

Your model will show up in the main window.

[[file:files/model_editor_car_done.png|800px]]

# Edit existing models

Rather than creating a model from the ground up with simple shapes and meshes,
you can also edit existing models that are already in the simulation.

To edit an existing model:

* Make sure you have saved the model you've created and are now back to the
normal gazebo mode. Alternatively, start from a fresh gazebo instance.

* Insert a model from the `Insert` tab of the left. For example, let's
insert a `Simple Arm`.

* Right click on the model you just inserted and select `Edit Model`.

[[file:files/model_editor_existing_model.png|800px]]

Now you are in the model editor mode and you are free to add new links to the
model or edit existing ones.
