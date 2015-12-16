# Model Editor

Now we'll construct our simple robot. We'll make a wheeled vehicle and add a sensor that will later allow us to make the robot follow a blob (person).

The Model Editor lets us construct simple models right in the GUI. For more complex models, you'll need to learn how to write SDF (XML-based) files [link to tutorial].
But for now, we can do everything right in the Gazebo GUI!


## Model Editor User Interface

To enter the Model Editor, click on `Edit` in the menu bar and select `Model Editor`. Or, use the hotkeys, Ctrl+M. **Physics and the simulation will be paused** as soon as you are in the Model Editor.

The Model Editor interface looks similar to the main Gazebo UI but with some subtle differences. The left panel and top Toolbar now contain only widgets for editing and creating parts of the model. The bottom Toolbar that displays simulation data is hidden since the simulation is now paused.

#### Left Panel

The left panel, also known as the **Palette**, has two tabs.

* **INSERT** tab: The Insert tab is where you add new parts (links and models)
  to the Model Editor. There are three sections.
      * Simple Shapes: These are primitive geometries that can be
      inserted to form a *link* of the model.
      * Custom Shapes: The `Add` button allows you to import custom meshes
      (currently supports COLLADA, STL, and SVG files) into the editor to form
      a *link* of the model.
      * Model Database: Located at the bottom half of the Palette is a
      list of *models*. These are models on your local computer or models
      available for download from the online model database.
      They can be inserted into the editor in the same way as simple shapes
      and custom meshes. Once inserted, they become a part of the model you are
      building. We refer to them as *Nested Models*.


* **Settings** tab: The Settings tab allows you to set the name and basic parameters of the model you
are building. It also displays a list of the links, joints, nested models, and plugins that are currently
part of the model. You can view and modify a part's parameters, like its pose, in two ways: 1) by double-clicking on the part in the list, or 2) by
right-clicking and selecting Open Inspector from the context menu in the Scene.

#### Toolbar

Like in Simulation mode, the main Toolbar in the Model Editor includes tools for interacting with the objects in the Scene (link to previous tutorial).
A new Joint Creation tool is available; it is used to create joints between links in the model.

#### Limitations

The Model Editor supports most of the basic model building tasks that can be done by writing SDF. However, there are still a few features that are not
yet available:

* adding and editing model plugins.

* editing nested models and links within nested models.

* adding and editing certain geometry types including Plane and Polyline.

* support for heightmaps.

* CAD functionalities.

## Vehicle construction

### Creating a vehicle

This section provides step-by-step instructions on creating a simple vehicle model in the Model Editor.

**Chassis**

1. First,  we'll create the vehicle chassis. In the Insert tab in the left panel, click once on the Box icon, move the cursor to anywhere in the Scene,
   and click again to release the box.

    [[file:files/ftu4/ftu4-editor_box.png|400px]]

1. Next, resize the box so that it looks more like the shape of a car chassis. We can do this using the Scale tool located on the top Toolbar.
   Select the box in the Scene, and a RGB-colored marker should appear over the box. The red color arrow represents the X axis, green is Y, and blue is Z. Move the mouse over the red arrow to highlight it, then click and drag to make the chassis longer along the X axis. Scale the chassis so it is roughly 2 meters long. You can estimate this by looking at the 1x1 meter grids on the ground.

    [[file:files/ftu4/ftu4-scale_tool.png|200px]]

1. Now flatten the chassis with the Scale tool. Click and drag the blue arrow down so that the chassis is approximately half of its original size.

    [[file:files/ftu4/ftu4-chassis_scale.png|600px]]

1. We want to lower the chassis closer to the ground. To give exact measurements, we will use the Link Inspector. Double-click on the box to bring up the
   Inspector. Scroll down to the bottom of the Link tab to find the `Pose` parameters and change `Z` to be 0.4m. Click `OK` to save the changes and close
   the Inspector.

    [[file:files/ftu4/ftu4-chassis_height.png|600px]]

**Front Wheels**

1. Let's move on to the front wheels. Start by inserting a cylinder from the Insert tab on the left panel.

1. The cylinder in its default orientation will not roll very well. Let's rotate it along the X axis using the Link Inspector. Double-click on the cylinder, scroll to the bottom, and change `Roll` to 1.5707 radians (90 degrees) and hit the Enter key on the keyboard. Do not close the Inspector just yet.

    [[file:files/ftu4/ftu4-wheel_rotate.png|600px]]

1. Next, resize the wheel by giving it exact dimensions. Go to the Visual tab to see the list of visuals in this link. There should only be one. Expand the visual item by clicking on the small arrow next to the `visual` text label. Scroll down to the `Geometry` section and change the `Radius` to 0.3m and `Length` to 0.25m. Hit Enter when done.

    [[file:files/ftu4/ftu4-wheel_visual.png|600px]]

1. You should now see in the Scene that a smaller cylinder appears inside the bigger cylinder. This is expected as we have only changed the visual geometry but not the collision. A 'visual' is the graphical representation of the link and does not affect the physics simulation. On the other hand, a 'collision' is used by the physics engine for collision checking. To also update the wheel's collision, go to the Collision tab, expand the only collision item, and enter the same Geometry dimensions. `Radius`: 0.3m and `Length`: 0.25m.

1. Now that we have created our first wheel, we'll use it as a template and make another one. Select the wheel and click on the Copy icon in the top Toolbar.
gm
    [[file:files/ftu4/ftu4-copy_tool.png|200px]]

1. Click on the Paste icon and move the mouse back to the Scene to insert the copy.

    [[file:files/ftu4/ftu4-wheel_paste.png|600px]]

1. The chassis and the wheels are currently free-moving bodies. To constrain their motion, we'll add joints between each wheel and the chassis. Begin by clicking on the Joint icon in the top Toolbar to bring up the Joint Creation dialog.

    [[file:files/ftu4/ftu4-joint_dialog.png|600px]]

1. The Joint Creation dialog contains joint properties that are commonly specified for a joint. Before configuring any of the properties, you are prompted to select the parent and child links of the joint. Move the mouse over the chassis to see it highlighted, and click on it to set it as the parent of the joint.

1. Move the mouse to the left front wheel; a line should now extend from the origin of the chassis to the end of the mouse. Click on the wheel to set it as the child of the joint. A new joint is created. By default it is a revolute joint (as indicated in under `Joint Types` section in the dialog) which just happens to be the joint type we want.

    Note: You may find it useful to change the view angle at this point. This can be done in the Upper Toolbar; click the cube icon with an orange side.

    [[file:files/ftu4/ftu4-wheel_joint.png|600px]]

1. Next, we need to configure the axis of rotation of the wheel. In the Joint Creation dialog. find the `Joint axis` section and change the axis to be `Z` (0, 0, 1). Pay attention to the RGB joint visual on the wheel. You should see that a yellow ring now appears over the blue arrow of the joint visual to indicate that it is the axis of rotation.

    [[file:files/ftu4/ftu4-wheel_rotation_axis.png|600px]]

1. To align the wheel next to the chassis, we will use the different alignment options in the `Align links` section in the Joint Creation dialog. First, we will align in the X axis so click on the `X Align Max` option to see the result of the alignment. The cylinder should be highlighted to indicate that its pose has changed.


    [[file:files/ftu4/ftu4-wheel_align_x.png|600px]]

1. In our example, we want to position the wheel flush against the chassis. To bring the wheel closer, click the `Y Align Max` option. However, it is not quite what we want yet. Click the `Reverse` option next the Y alignment options to align the wheel's minimum (reverse of maximum) to the chassis's maximum. Note that the `Reverse` option is applied to the child link since that the default alignment configuration shown in the drop down list below is `Child to Parent`.  If `Parent to Child` configuration is set, the `Reverse` option will be applied to the parent link.

    [[file:files/ftu4/ftu4-wheel_align_y_reverse.png|600px]]

1. To position the wheel above the ground, we can use the `Relative Pose` section at the bottom of the dialog to move the child link (wheel) relative to the parent link (chassis). Given that the wheel has a radius of 0.3m and the chassis is at 0.4m above the ground, simple math calculation will show that we need to place the wheel at -0.1m relative to the chassis. So go ahead and change the `Z` position to -0.1m.

    [[file:files/ftu4/ftu4-wheel_pose_z.png|600px]]

1. Press the `Create` button when done. This will finalize the joint and close the Joint Creation dialog.

1. Repeat the joint creation process and axis configuration for the other front wheel, make sure that a) the chassis is the parent of the joint and the wheel is the child, b) the axis of rotation is set to `Z`, and c) use the `Y Align Min` option to align the right wheel as it is on the other side of the chassis.

    [[file:files/ftu4/ftu4-wheel_joints.png|600px]]

**Caster Wheel**

1. To make a caster wheel for the vehicle, click on the Sphere button on left panel and insert it into the scene.

    [[file:files/ftu4/ftu4-caster_sphere.png|600px]]

1. Resize the sphere by giving it exact dimensions in the same way as you did for the front wheels. Go to the Visual tab to see the list of visuals in this link, expand the only visual item, scroll down to the `Geometry` section and change the `Radius` to 0.2m. Make sure to also do the same to the collision in the Collision tab.

    [[file:files/ftu4/ftu4-caster_resize.png|600px]]

1. To create a joint between the caster wheel and the chassis, bring up the Joint Creation dialog by clicking on the Joint icon in the top toolbar. Move the mouse to the scene and select the chassis as the parent link and the sphere as the child link.

    [[file:files/ftu4/ftu4-caster_joint.png|600px]]

1. Unlike the front wheel joints, a caster wheel rolls in all directions and does not have a specific axis of rotation. In Gazebo, this is simulated using a ball joint. So under the `Joint types` section, select the `Ball` joint option. You should see the joint visual in the scene change color to indicate a different joint type has been set.

    [[file:files/ftu4/ftu4-caster_joint_ball.png|600px]]

1. Next align the caster wheel so that it is centered with the chassis and positioned at the rear end. In the Align links section, select the `Y Align Center` option to center the two links in the `Y` axis, and select the `X Align Min` option to move the caster wheel so it is placed right at the back of the vehicle.

    [[file:files/ftu4/ftu4-caster_align.png|600px]]

1. Finally, position the caster wheel so that it sits just above the ground. Do this by setting the `Z` position in the Relative Pose section to -0.3m.

    [[file:files/ftu4/ftu4-caster_joint_z.png|600px]]

1. Press the `Create` button to finish the joint creation process.

**Sensor**


### Adding a plugin

(add a blob tracker to the front; explain that plugins are usually more complex and require code; link to Plugins "Topic of Interest")

### Save your model

1. Save the model by going to the `File` menu and select `Save Model`. Enter a name for the model and click `Save`.

    [[file:files/ftu4/ftu4-vehicle_save.png|600px]]

1. Exit the Model Editor by going to `File` and selecting `Exit Model Editor`. Gazebo should now switch back to normal simulation mode. Hit the Play button to
   continue running the simulation. If you want to edit the model again later, just right-click on it and select `Edit Model` from the context menu.

    [[file:files/ftu4/ftu4-vehicle_done.png|600px]]


1. Feel free to play around with the Joint Controller on the right panel to move the vehicle.
