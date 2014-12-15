This tutorial describes the process of creating a building using the Building Editor.

# Overview

## Open the Building Editor

1.  Make sure Gazebo is [installed](http://gazebosim.org/tutorials?cat=installation).

1.  Start up gazebo.

    ~~~
    $ gazebo
    ~~~

1. On the `Edit` menu, go to `Building Editor`, or hit `Ctrl+B` to open the editor.

    [[file:files/empty_editor.png|800px]]

## Graphical user interface

The editor is composed of the following 3 areas:

1. The **Palette**, where you can choose features and materials for your building.

1. The **2D View**, where you can import a floor plan to trace over (optional) and insert walls, windows, doors and stairs.

1. The **3D View**, where you can see a preview of your building. It is also where you can assign colors and textures to different parts of your building.

    [[file:files/editor_zones.png|800px]]

# Import a floor plan

You may create a scene from scratch, or use an existing image as a template to trace over. This image can be, for example, a 2D laser scan of a building.

Click [here](https://bitbucket.org/osrf/gazebo_tutorials/raw/default/building_editor/files/floorplan.png) to get an example floor plan, then proceed as follows:

1. Click on the `Import` button. The `Import Image` dialog will come up.

1. Step 1: Choose the image you previously saved on your computer and click `Next`.

    [[file:files/import_step_1.png|800px]]

1. Step 2: To make sure the walls you trace over the image come up in the correct scale, you must set the image's resolution in pixels per meter (`px/m`). If we knew the resolution, we could directly type it in the dialog and click `Ok`. In this example we don't know the resolution, but we know the real-world distance between two points in the image (for example, the top wall of 7.5 m), so we can use that to calculate the resolution:

    a. Click/release on one end of the wall. As you move the mouse, an orange line will appear as shown below.

    b. Click/release at the end of the wall to complete the line.

    c. Now type the distance in meters in the dialog (7.5 m in this case). The resolution will be automatically calculated for you based on the line you drew.

    d. You can then click `Ok`.

    [[file:files/import_step_2.png|800px]]

1. The image will appear on the 2D View properly scaled.

    > **Tip:** Once you've added more levels, you can import a floor plan for each by repeating the same process.

# Add features

## Add walls

Trace all walls on the floor plan as follows. Keep in mind that we will attach windows and doors to the walls later, so here you can draw the walls over them. Don't worry too much if the walls are not perfect, we will edit them later.

1. On the Palette, click on `Wall`.

1. On the 2D View, click/release anywhere to start the wall. As you move the mouse, the wall's length is displayed.

1. Click again to end the current wall and start an adjacent wall.

1. Double-click to finish a wall without starting a new one.

    > **Tip:** You can right-click or press `Esc` to cancel drawing the current wall segment.

    > **Tip:** By default, walls snap to 15° and 0.25 m increments and also to the end points of existing walls. To override this, hold `Shift` while drawing.

    [[file:files/add_walls.png|800px]]

## Add windows and doors

**Note: Currently, windows and doors are simple holes in the wall.**

Let's insert windows and doors at the locations shown on the floor plan.

1. On the Palette, click on `Window` or `Door`.

2. As you move the mouse in the 2D view, the feature to be inserted moves with it, as does its counterpart in the 3D View.

    > **Tip:** Windows and doors automatically snap to walls as you hover over them. The distances to the ends of the wall are displayed as you move.

3. Click on the desired position to place the feature.

    [[file:files/add_windows_doors.png|800px]]

    > **Tip:** It might be difficult to see where the features are on your floor plan after the walls have been drawn on top of it. To make it easier, at the top of the 2D View, you can choose to view or hide the floor plan or features for the current level. You can also use hotkeys to toggle visibility, `F` for floor plan and `G` for features.

    [[file:files/view_floorplan.png|640px]]

## Add stairs

There are no staircases on this floor plan, but we will insert one anyways.

1. On the Palette, click on `Stairs`.

1. As you move the mouse in the 2D view, the staircase to be inserted moves with it, as does its counterpart in the 3D View.

1. Choose a position for your staircase and click to place it.

    [[file:files/add_stairs.png|800px]]

## Add levels

We're pretty much done with Level 1. Let's add another level to our building so our staircase ends up somewhere.

At the top of the 2D View, click on `+` to add a level. Alternatively, right-click the 2D View and choose `Add a level`.

When a new level is added, a floor is automatically inserted. If there are stairs on the level below, a hole above the stairs will be cut out from the floor when the building is saved.

> **Note: Currently, all floors are rectangular.**

> **Tip:** Before adding a level, make sure you have walls on the current level to build on top of.

> **Tip:** Currently, all the walls from the level below are copied to the new level, with default materials. No other features are copied. You can manually delete the walls you don't want. 

[[file:files/add_level.png|800px]]

# Edit your building

**Note: Be careful when editing your building; the editor currently has no option to undo your actions.**

> **Tip:** All measurements are in meters.

## Change levels

Since we added a level, we were brought to the new level in the 2D view. You can go back to Level 1 by choosing it from the drop-down list at the top of the 2D View.

> **Tip:** The level currently selected in the 2D View will appear as semi-transparent in the 3D View and all levels below it will appear opaque. Levels above will be hidden - but keep in mind they are still part of your building!

We can also edit some level configurations if we want.

* Double-click the 2D View to open an inspector with level configuration options. Alternatively, right-click and choose `Open Level Inspector`.

You may have added levels that you don't want, or perhaps made a mess in the current level and would like to start it over.

* To delete the current level, either press the `-` button at the top of the 2D View, or right-click and choose `Delete Level`.

    [[file:files/edit_level.png|640px]]

## Edit walls

We drew a lot of walls earlier, but maybe they didn't turn out exactly the way we wanted.

* In the 2D View, click on the wall to be edited.

    a. Translate the wall by dragging it to a new position.

    b. Resize or rotate the wall by dragging one of its end points.

    > **Tip:** By default, walls snap to 15° and 0.25 m increments. To override this, hold `Shift` while drawing.

* Double-click a wall in the 2D View to open an inspector with configuration options. Alternatively, right-click and choose `Open Wall Inspector`. Edit some fields and press `Apply` to preview the changes.

* To delete a wall, either press the `Delete` key while it is selected, or right-click the wall in the 2D View and choose `Delete`.

    > **Tip:** Editing a wall takes attached walls into account.

    > **Tip:** Deleting a wall deletes all doors and windows attached to it.

    [[file:files/edit_walls.png|640px]]

## Edit windows and doors

Now let's play around with windows and doors. As we did for the walls, we can manipulate windows and doors more precisely in a few different ways. 

* In the 2D View, click on the feature to be edited.

    a. Translate the feature by dragging it to a new position. Remember that windows and doors automatically snap to walls and it doesn't make much sense to have them detached from any walls, as they represent holes in a wall.

    b. Rotate the feature by dragging its rotation handle. Currently, as long as they are attached to a wall, their orientation doesn't make a difference.

    c. Resize the feature's width by dragging one of the end points.

* Double-click a feature in the 2D View to open an inspector with configuration options. Alternatively, right-click and choose `Open Window/Door Inspector`.

* To delete a feature, either press the `Delete` key while it is selected, or right-click it in the 2D View and choose `Delete`.

    [[file:files/edit_windows_doors.png|640px]]

## Edit stairs

Finally, let's edit the staircase we inserted earlier. Since it is not on the floor plan, we can get creative and resize it as we want.

* In the 2D View, click on the staircase to select it.

    a. Translate the staircase by dragging it to a new position.

    b. Rotate the staircase in multiples of 90° by dragging its rotation handle.

    c. Resize the staircase by dragging one of the end nodes.

* Double-click the staircase in the 2D View to open an inspector with configuration options. Alternatively, right-click and choose `Open Stairs Inspector`.

* To delete the staircase, either press the `Delete` key while it is selected, or right-click and choose `Delete`.

> **Tip:** In the 2D View, staircases are visible on both the start and end levels.

[[file:files/edit_stairs.png|640px]]

## Add colors and textures

Now that everything is properly placed and sized, you can assign colors and textures to walls, floors and staircases. Remember that windows and doors are only holes on the wall and therefore cannot have materials.

> **Tip:** The default color is white and the default texture is none.

There are two ways to add colors and textures to your building:

### From Inspectors

You can add color and texture to walls, stairs and floors from the `Wall Inspector`, `Stairs Inspector` and `Level Inspector` respectively. Simply open the inspector, select your materials and press `Apply`.

[[file:files/color_texture_inspector.png|640px]]

### From the Palette

Colors and textures can be chosen from the Palette and assigned to items on your building by clicking on them in the 3D View.

1. Click on a color or texture in the Palette.

1. As you move your mouse in the 3D View, hovered features will be highlighted displaying a preview of the selected material.

1. Clicking on the highlighted feature assigns the selected material to it. You can click on as many features as you'd like.

1. When you're done with the selected material, either right-click the 3D view, or click outside any features to leave the material mode.

    [[file:files/color_texture_palette.png|800px]]

1. To choose a custom color, click on `More` on the Palette. A dialog opens where you can specify custom colors.

    [[file:files/custom_color.png|640px]]

> **Tip:** Each feature can have only one color and one texture. The same material is assigned to all faces of the feature.

> **Note: Currently, it is not possible to assign custom textures on the Building Editor.**

# Saving your building

Saving will create a [directory, SDF and config files](http://gazebosim.org/tutorials?tut=model_structure&cat=build_robot) for your building.

Before saving, give your building a name on the Palette.

[[file:files/edit_name.png|200px]]

On the top menu, choose `File`, then `Save As` (or hit `Ctrl+S`). A dialog will come up where you can choose the location for your model.

> **Tip:** Under `Advanced Options` you can set some meta-data for your building.

[[file:files/save_dialog.png|300px]]

# Exit

**Note: Once you exit the Building Editor, your building will no longer be editable.**

When you're done creating your building and you've saved it, go to `File` and then `Exit Building Editor`.

Your building will show up in the main window. In the future, you can find the building in your `Insert` tab.

[[file:files/saved_building.png|800px]]

[[file:files/final_model_angles.png|800px]]
