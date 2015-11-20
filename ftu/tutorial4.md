# Model Editor

Now we'll construct our simple robot. We'll make a wheeled vehicle and add a sensor that will later allow us to make the robot follow a blob (person).

The Model Editor lets us construct simple models right in the GUI. For more complex models, you'll need to learn how to write SDF (XML-based) files [link to tutorial].
But for now, we can do everything right in the Gazebo GUI!


## Model Editor User Interface

To enter the Model Editor, click on `Edit` in the menu bar and select `Model Editor`. Or, use the hotkeys, Ctrl+M. The simulation will be paused as soon as you are in the Model Editor.

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

1. First,  we'll create the vehicle chassis. In the Insert tab in the left panel, click once on the Box icon, move the cursor to anywhere in the Scene,
   and click again to release the box.

1. Next, resize the box so that it looks more like the shape of a car chassis. We can do this using the Scale tool located on the top Toolbar.
   Select the box in the Scene, and a RGB-colored marker should appear over the box. The red color arrow represents the X axis, green is Y, and blue is Z.
   Move the mouse over the red arrow to highlight it, then click and drag to make the chassis longer along the X axis. Scale the chassis so it is roughly 2
   meters long. You can estimate this by looking at the 1x1 meter grids on the ground.

1. Now flatten the chassis with the Scale tool. Click and drag the blue arrow down so that the chassis is approximately half of its original size.

    [[file:files/ftu4/ftu4-chassis_scale.png|600px]]

1. We want to lower the chassis closer to the ground. To give exact measurements, we will use the Link Inspector. Double-click on the box to bring up the
   Inspsector. Scroll down to the bottom of the Link tab to find the `Pose` parameters and change `Z` to be 0.4m. Click `OK` to save the changes and close
   the Inspector.

1. Let's move on to the wheels. Start by inserting a cylinder from the Insert tab on the left panel.

1. The cylinder in its default orientation will not roll very well. Let's rotate it along the X axis using the Link Inspector. Double-click on the cylinder,
   scroll to the bottom, and change `Roll` to 1.5707 radians (90 degrees) and hit the Enter key on the keyboard. Do not close the Inspector just yet.

    [[file:files/ftu4/ftu4-wheel_rotate.png|600px]]

1. Next, resize the wheel by giving it exact dimensions. Go to the Visual tab to see the list of visuals in this link. There should only be one. Expand the
   visual item by clicking on the small arrow next to the `visual` text label. Scroll down to the `Geometry` section and change the
   `Radius` to 0.3m and `Length` to 0.25m. Hit Enter when done.

    [[file:files/ftu4/ftu4-wheel_visual.png|600px]]

1. You should now see in the Scene that a smaller cylinder appears inside the bigger cylinder. This is expected as we have only changed the visual geometry
   but not the collision. A 'visual' is the graphical representation of the link and does not affect the physics simulation. On the other hand, a 'collision' is used by the physics engine for collision checking. To also update the wheel's collision, go to the Collision tab, expand the only collision item, and enter the same Geometry dimensions. `Radius`: 0.3m and `Length`:
   0.25m.

1. Now that we have created our first wheel, we'll use it as a template and make three more. Select the wheel and click on the Copy icon in the top Toolbar.

    [[file:files/ftu4/ftu4-wheel_copy.png|600px]]

1. Click on the Paste icon and move the mouse back to the Scene to insert the copy. Repeat the process until there are four wheels in the Scene.

1. The chassis and the wheels are currently free-moving bodies. To constrain their motion, we'll add joints between each wheel and the chassis. Begin by
   clicking on the Joint Creation icon in the top Toolbar. Move the mouse over the chassis to see it highlighted, and click on it to set it as the parent
   of the joint.

1. Move the mouse to one of the wheels; a line should now extend from the origin of the chassis to the end of the mouse. Click on the wheel to set it as
   the child of the joint. A new joint is created. By default it is a revolute joint which just happens to be the joint type we want.

    [[file:files/ftu4/ftu4-wheel_joint.png|600px]]

1. Next, we need to configure the axis of rotation of the wheel. Double-click on the joint (orange line) that connects the chassis and the
   wheel to bring up the Joint Inspector. Scroll down to `Axis1`'s `Xyz` parameter, and change the axis to be `Z` (0, 0, 1). Pay attention to the RGB
   joint visual on the wheel. You should see that a yellow ring now apears over the blue arrow of the joint visual to indicate that it is the axis of
   rotation.

    [[file:files/ftu4/ftu4-wheel_copy.png|600px]]

1. Repeat the joint creation process and axis configuration for the remaining three wheels, make sure that a) the chassis is the parent of the joint and the
   wheel is the child, and b) the axis of rotation is set to Z.

1. To position the wheel next to the chassis we'll use two tools. First, we will use the Snap tool to position the wheel flush against the chassis.
   Select the Snap icon in the top Toolbar and click on the flat side of the cylinder to indicate that it will be moved during the snap action.

    [[file:files/ftu4/ftu4-wheel_snap.png|600px]]

1. Orient the user camera to see the side of the chassis and click on it to attach the wheel to the chassis. Do not worry about its position for now.

1. To align the wheel to the chassis, we will use the Align tool. Press Escape to switch to Select mode, and click
   on the chassis to select it. Hold the Ctrl key to enable multi-selection and click on the wheel that is attached to it - you should see that both objects
   are now selected.

1. Click on the Align tool to bring up a sub-menu consisting of different alignment options. In our example, we want to align in the X axis so hover over
   either the `X Align Max` or `X Align Min` option to see a preview of the alignment result. The cylinder should be aligned to the front or rear end of
   the box. Click on the the alignment option to fix the configuration.

1. To position the cylinder back above the ground, open the Link Inspector, scroll down to the `Pose` section, and change the `Z` position to 0.3m
   (same value as the cylinder radius).

1. Repeat the Snap and Alignment process for the remaining three wheels. Make sure to set the wheel position so that it sits on the ground. Once done,
   we have a vehicle with differential drive capability!


### Adding a plugin

(add a blob tracker to the front; explain that plugins are usually more complex and require code; link to Plugins "Topic of Interest")

### Save your model

1. Save the model by going to the `File` menu and select `Save Model`. Enter a name for the model and click `Save`.

1. Exit the Model Editor by going to `File` and selecting `Exit Model Editor`. Gazebo should now switch back to normal simulation mode. Hit the Play button to
   continue running the simulation. If you want to edit the model again later, just right-click on it and select `Edit Model` from the context menu.

1. Feel free to play around with the Joint Controller on the right panel to move the vehicle.
