This tutorial describes the process of creating a building using the Building Editor.

# Overview

## Open the Building Editor

1.  Make sure Gazebo is [installed](http://gazebosim.org/tutorials?cat=installation).

1.  Start up gazebo.

    ~~~
    $ gazebo
    ~~~

1. On the top menu, go to `Edit` and then `Building Editor`, or hit `Ctrl+B` to open the editor.

    [[file:files/empty_editor.png|800px]]

## Graphical user interface

The editor is composed of the following 3 areas:

1. The **palette**, where you can choose features and materials for your building.

1. The **2D View**, where you can import a floor plan to trace over and insert walls, windows, doors and stairs.

1. The **3D view**, where you can see a preview of how your building will look like once it is saved. It is also where you can assign colors and textures to different parts of your building.

    [[file:files/editor_zones.png|800px]]

# Import a floor plan

You may create a scene from scratch, or use an existing image as a template to trace over. This image can be, for example, a 2D laser scan of a building.

You can import an existing floor plan for each level of your building as follows.

1. Click on the `Import` button. The `Import Image` dialog will come up.

1. Step 1: Choose a `jpg` or `png` image from your computer and click `Next`.

    [[file:files/import_step_1.png|800px]]

1. Step 1: To make sure the walls you trace over the image come up in the correct scale, you must set the image's resolution in pixels per meter (`px/m`). There are two ways of setting the resolution:

    a. If you know your image's resolution, you can directly input the resolution on the dialog and click `Ok`.

    b. If you don't know the resolution, but you know the real-world distance of two points in the image, you can click on the two points to draw a line on the image and input the line's length in meters on the dialog. The resolution will be automatically calculated for you based on the line you drew (as an example, see the orange line of 7.5 m below). You can then click `Ok`.

    [[file:files/import_step_2.png|800px]]

1. The image will appear on the 2D View properly scaled.

> Tip: You can add a floor plan for each level on your building once you've added more levels, by repeating the same process for each level.

# Add features

## Add walls

1. On the palette, click on `Wall`.

1. On the 2D View, click anywhere to start wall. As you move the mouse, the wall's length is displayed.

1. Click again to end the current wall and start another wall attached to it.

1. Double-click to end a wall without starting a new one.

    > Tip: You can also right-click or press `Esc` to cancel drawing the current wall segment.

    > Tip: By default, walls snap to 15° and 0.25 m increments and also to the end points of existing walls. To override this, hold `Shift` while drawing.

    [[file:files/add_walls.png|800px]]

## Add windows and doors

**Note: Currently, windows and doors are simple holes on the wall.**

1. On the palette, click on `Window` or `Door`.

2. As you move the mouse on the 2D view, the feature to be inserted moves with it, as does its counterpart on the 3D View.

3. Click on the desired position to place the feature.

    > Tip: Windows and doors automatically snap to walls as you hover over them. The distances to the ends of the wall are displayed as you move.

    [[file:files/add_windows_doors.png|800px]]

## Add stairs

1. On the palette, click on `Stairs`.

1. As you move the mouse on the 2D view, the staircase to be inserted moves with it, as does its counterpart on the 3D View.

1. Click on the desired position to place the staircase.

    [[file:files/add_stairs.png|800px]]

## Add levels

> Tip: Before adding a level, make sure you have walls on the current level to build on top of.

On the top of the 2D View, click on `+` to add a level. You can also right-click the 2D View and choose `Add a level`.

When a new level is added, a floor is automatically inserted. If there are stairs on the level below, a hole above the stairs will be cutout from the floor when the building is saved.

> **Note: Currently, all floors are rectangular.**

> Tip: All the walls from the level below are copied to the new level, with default materials. No other features are copied.

[[file:files/add_level.png|800px]]

# Edit features

**Note: Be careful when editing your building, the editor currently has no option to undo your actions.**

> Tip: All measurements are in meters.

## Edit walls

* On the 2D View, click on the wall to be edited.

    a. Translate the wall by dragging it to a new position.

    b. Resize or rotate the wall by dragging one of its end points.

    > Tip: By default, walls snap to 15° and 0.25 m increments. To override this, hold `Shift` while drawing.

* Double-click a wall on the 2D View to open an inspector with configuration options. You can also right-click and choose `Open Wall Inspector`.

* To delete a wall, either press the `Delete` key while it is selected, or right-click it on the 2D View and choose `Delete`.

    > Tip: Editing a wall takes attached walls into account.

    > Tip: Deleting a wall deletes all doors and windows attached to it.

    [[file:files/edit_walls.png|640px]]

## Edit windows and doors

* On the 2D View, click on the feature to be edited.

    a. Translate the feature by dragging it to a new position. Windows and doors automatically snap to walls.

    b. Rotate the feature by dragging its rotation handle.

    c. Resize the feature's width by dragging one of the end points.

* Double-click a feature on the 2D View to open an inspector with configuration options. You can also right-click and choose `Open Window/Door Inspector`.

* To delete a feature, either press the `Delete` key while it is selected, or right-click it on the 2D View and choose `Delete`.

    [[file:files/edit_windows_doors.png|640px]]

## Edit stairs

* On the 2D View, click on the staircase to be edited.

    a. Translate the staircase by dragging it to a new position.

    b. Rotate the staircase to multiples of 90° by dragging its rotation handle.

    c. Resize the staircase by dragging one of the end nodes.

* Double-click a staircase on the 2D View to open an inspector with configuration options. You can also right-click and choose `Open Stairs Inspector`.

* To delete a staircase, either press the `Delete` key while it is selected, or right-click and choose `Delete`.

> Tip: On the 2D View, staircases are visible on both the level it starts at and the level it ends at.

    [[file:files/edit_stairs.png|640px]]

## Edit levels

* You can choose what level to see in the 2D View from the drop-down list on the top.

    > Tip: The level currently selected in the 2D View will appear as semi-transparent on the 3D View, all levels below it will appear opaque and levels above will be hidden - but keep in mind it is still part of your building!

* Double-click the 2D View to open an inspector with level configuration options. You can also right-click and choose `Open Level Inspector`.

* To delete the current level, either press the `-` button on the top of the 2D View, or right-click and choose `Delete Level`.

    [[file:files/edit_level.png|640px]]

> Tip: On the top of the 2D View, you can choose to view or hide the floor plan or features for the current level.

    [[file:files/view_floorplan.png|640px]]


## Add colors and textures

You can assign colors and textures to walls, floors and staircases. Currently, windows and doors are only holes on the wall and therefore cannot have materials.

> Tip: The default color is white and the default texture is none.

There are two ways to add colors and textures to your building:

### From inspectors

You can add color and texture to walls, stairs and floors from the Wall Inspector, Stairs Inspector and Level Inspector respectively. Simply open the inspector, select your materials and press `Apply`.

[[file:files/color_texture_inspector.png|640px]]

### From the palette

Colors and textures can be chosen from the palette and assigned to items on your building by clicking on them on the 3D View.

1. Click on a color or texture on the palette.

1. As you move your mouse on the 3D View, hovered features will be highlighted displaying a preview of the selected material.

1. Clicking on the highlighted feature assigns the selected material to it. You can click on as many features as you'd like.

1. When you're done with the selected material, either right-click the 3D view, or click outside any features to leave the material mode.

    [[file:files/color_texture_palette.png|800px]]

1. To choose a custom color, click on `More` on the palette. A dialog opens where you can specify custom colors.

    [[file:files/custom_color.png|640px]]

> Tip: Each feature can have only one color and one texture. The same material is assigned to all faces of the feature.

> Note: Currently, it is not possible to assign custom textures from the GUI.

# Saving your building

Saving will create a [directory, SDF and config files](http://gazebosim.org/tutorials?tut=model_structure&cat=build_robot) for your building.

Before saving, give your building a name on the palette.

[[file:files/edit_name.png|200px]]

On the top menu, choose `File`, then `Save As` (or hit `Ctrl+S`). A dialog will come up where you can choose the location for your model.

> Tip: Under `Advanced Options` you can set some meta-data for your building.

[[file:files/save_dialog.png|300px]]

# Exit

**Note: Once you exit the Building Editor, your building will no longer be editable.**

When you're done creating your building and you've saved it, go on `File` and then `Exit Building Editor`.

Your building will show up on the main window. And in the future, you can find it on your `Insert` tab.

[[file:files/saved_building.png|800px]]

[[file:files/final_model_angles.png|800px]]
