This tutorial describes the process of creating a building using the Building Editor.

# Open the Building Editor

1.  Make sure Gazebo is [installed](http://gazebosim.org/tutorials?cat=installation).

1.  Start up gazebo.

    ~~~
    $ gazebo
    ~~~

1. On the top menu, go to `Edit` and then `Building Editor`, or hit `Ctrl+B` to open the editor.

[[file:files/empty_editor.png|640px]]

# Graphical user interface

The editor is composed of the following 3 areas:

1. The **palette**, where you can choose features and materials for your building.

1. The **2D View**, where you can import a floorplan and add walls, windows, doors and stairs.

1. The **3D view**, where you can see a preview of how your building will look like once it is saved. It is also where you can assign colors and textures to different parts of your building.

[[file:files/editor_zones.png|640px]]

# Import a floor plan

Users may create a scene from scratch, or they may use an image as a template to trace over. This image can be, for example, a 2D laser scan of a building.

You can import an existing floor plan for each level of your building as follows.

1. Make sure you're on the right level and click on the `Import` button.

1. Choose a jpg or png image from your computer and click `Next`.

[[file:files/import_step_1.png|640px]]

1. To make sure your building will be in the right scale, you must set its resolution in pixels per meter (px/m). There are two ways of setting the resolution:

    a. If you know your image's resolution, you can directly input the resolution on the dialog and click Ok.

    b. If you don't know the resolution, but you know the real-world distance of two points in the image, you can click on the two points and input the distance in meters on the dialog. The resolution will be automatically calculated for you based on the line you drew.

[[file:files/import_step_2.png|640px]]

# Add features

## Add walls

* On the palette, click on `Wall`.

* On the 2D View, click to start wall. As you move the mouse, the wall's length is displayed.

* Click again to end wall and continue next wall attached to it.

* Double-click to end a wall without starting a new one.

* You can also right-click or press `ESC` to cancel drawing the current wall segment.

* By default, walls snap to 15° and 0.25 m increments and also to the end nodes of existing walls. To override this, hold `Shift` while drawing.

[[file:files/add_walls.png|640px]]

## Add windows and doors

* On the palette, click on `Window` or `Door`.

* As you move the mouse on the 2D view, the feature moves with it, and its 3D representation also moves accordingly.

* Windows and doors automatically snap to walls as you hover over them. The distances to the ends of the wall can also be seen.

* Click on the desired position to place the feature.

**Note: Currently, windows and doors are simple holes on the wall.**

[[file:files/add_windows_doors.png|640px]]

## Add stairs

* On the palette, click on `Stairs`.

* As you move the mouse on the 2D view, the feature moves with it, and its 3D representation also moves accordingly.

* Click on the desired position to place the staircase.

[[file:files/add_stairs.png|640px]]

## Add levels

* In order to add a level, first make sure you have walls on the current level to build on top of.

* On the top of the 2D View, click on `+` to add a level. You can also right-click the 2D View and choose `Add a level`.

* When a new level is added, a floor is automatically inserted.

    > Note that currently all floors are rectangular.

    > If there are stairs on the level below, a hole above the stairs will be cutout from the floor when the building is saved.

* All the walls from the level below are copied to the new level, but no other features.

[[file:files/add_level.png|640px]]

# Edit features

Be careful when editing your building, the editor currently has no option to undo your actions.

## Edit walls

* On the 2D View, click on the wall to be edited.

* Translate the wall by dragging it to a new position.

* Resize or rotate the wall by dragging one of the end nodes. By default, walls snap to 15° and 0.25 m increments. To override this, hold `Shift` while drawing.

* Editing a wall takes attached walls into account.

* Double-click a wall to open an inspector with configuration options. You can also right-click and choose `Open Wall Inspector`.

* To delete a wall, either press the `Delete` key while it is selected, or right-click and choose `Delete`.

[[file:files/edit_walls.png|640px]]

## Edit windows and doors

* On the 2D View, click on the feature to be edited.

* Translate the feature by dragging it to a new position. Windows and doors automatically snap to walls.

* Resize the feature's width by dragging one of the end nodes.

* Double-click a feature to open an inspector with configuration options. You can also right-click and choose `Open Window/Door Inspector`.

* To delete a feature, either press the `Delete` key while it is selected, or right-click and choose `Delete`.

[[file:files/edit_windows_doors.png|640px]]

## Edit stairs

* On the 2D View, click on the staircase to be edited.

* Translate the staircase by dragging it to a new position.

* Rotate the staircase to multiples of 90° by dragging the rotation handle.

* Resize the staircase by dragging one of the end nodes.

* Double-click a staircase to open an inspector with configuration options. You can also right-click and choose `Open Stairs Inspector`.

* To delete a staircase, either press the `Delete` key while it is selected, or right-click and choose `Delete`.

[[file:files/edit_stairs.png|640px]]

## Edit levels

* You can choose what level to be in from the drop-down list on the top of the 2D View.

* Double-click the 2D View to open an inspector with level configuration options. You can also right-click and choose `Open Level Inspector`.

* To delete the current level, either press the `-` button on the top of the 2D View, or right-click and choose `Delete Level`.

[[file:files/edit_level.png|640px]]

* On the top of the 2D View, you can choose to view or hide the floorplan or features for the current level.

[[file:files/view_floorplan.png|640px]]


## Add colors and textures

* The default color is white and the default texture is none.

* There are two ways to add colors and textures to your building:

1. Inspectors

You can add color and texture to walls, stairs and floors from the Wall Inspector, Stairs Inpector and Level Inspector respectively.

[[file:files/color_texture_inspector.png|640px]]

1. Palette

    a. Click on a color or texture on the palette.

    b. As you move your mouse on the 3D View, hovered features will be highlighted displaying a preview of the selected material.

    c. Clicking on the highlighted feature assigns the selected material to it. You can click on as many features as you'd like.

    d. When you're done with the material, either right-click the 3D view, or click outside any features.

    e. Each feature can have a color and a texture at a time. The same material is assigned to all faces of the feature.

    [[file:files/color_texture_palette.png|640px]]

    f. To choose a custom color, click on `More`. A dialog opens where you can specify custom colors.

    [[file:files/custom_color.png|640px]]

    g. Currently, it is not possible to assign custom textures from the GUI.

# Saving your building

Saving will create a [directory, SDF and config files](http://gazebosim.org/tutorials?tut=model_structure&cat=build_robot) for your building.

* Before saving, give your building a name on the palette.

[[file:files/edit_name.png|640px]]

* On the top menu, click `File`, then `Save As` (or hit `Ctrl+S`). A dialog will come up where you can choose the location for your model.

* Under advanced options you can set some meta-data for your building.

[[file:files/save_dialog.png|640px]]

# Exit

**Note: Once you exit the Building Editor, your building will no longer be editable.**

When you're done creating your building and you've saved it, go on `File` and then `Exit Building Editor`.

Your building will show up on the main window. And in the future, you can find it on your `Insert` tab.

[[file:files/saved_building.png|640px]]

[[file:files/final-building_angles.png|640px]]
