# Importing 2D models

Now that we've made a basic robot, we may want to pimp our ride. Sometimes
 it's easier to design part of a model in a program like Inkscape or
 Illustrator. This tutorial will show you how to make a custom wheel as an
 .svg in Inkscape, import it into Gazebo, and attach it to our robot.

In this tutorial, we are going to use an SVG editor to create the outlines
of the wheel. Then, if the outlines are closed and do not cross each other,
we will create a triangulation of the top side of the wheel, and extrude it
verically to create a 3D mesh.

## Using an SVG editor to draw a wheel

There are many SVG editors. For this tutorial, we will use the Open Source
 Inkscape program (see
[installation instructions](https://inkscape.org/en/download) ).

### Inkscape preparation

Start Inkscape. This will create a blank document. First, lets change the
 document size to better accomodate our wheel: under the `File->Document
 properties menu`, select the `Page` tab and change the document size to a
 custom size of 100.0 x 100.0 mm.

[[file:files/ftu5-inkscape-page-tab.png|800px]]

Then, in the same dialog, select the `Grids`
 tab, press the `New` button to create a custom grid. Then, check the`Enabled`,
`Visible`, and `Snap to visible grid lines only` options.

[[file:files/ftu5-inkscape-grids-tab.png|800px]]

You should end up with a document looking like this:

[[file:files/ftu5-inkscape-blank.png|800px]]

### Draw

You can use the different tools (pen, text, stars and shapes, etc...) to create
 your geometry. In this example, the wheel is made from circles (pressing the
 Shift Key you can start your circle from the center, and using the CTRL key
 allows you to keep the roudness of the shape). It is possible to combine
 shapes together, the only rule is that the paths must be closed and the
 part should have thickness. For example, a stick figure or two circles that
 touch each other would not result in valid Gazebo models.

[[file:files/ftu5-inkscape-simple-wheel.png|800px]]

Gazebo only imports `paths`, but it easy with Inkscape to transform any shape
 to a path. Select `Select All` from the `Edit` menu. Then select
`Path -> Object to Path` menu item. This will transform every object into
 separate paths and subpaths. This transformation is irreversible, so if you
 transform text into paths, you will not be able to alter the text.

[[file:files/ftu5-inkscape-select-all.png|800px]]

### Create a Gazebo Model

[SDFormat](http://sdformat.org) does not support SVG directly, it supports 2D
 polylines. The Gazebo Model Editor has an import mechanism that extract the
 polylines from SVG files, and save it them an SDF model file.


Launch Gazebo and Select `Model Editor` from the `Edit` menu to enter the
 Gazebo Model Editor mode (as opposed to the simulation mode). Then, set the
 thickness of the wheel to 0.025 m, and press `OK`. Your new link should
 appear in the 3D view.
The dialog also allows you to set the number of segments for each SVG spline.

[[file:files/ftu5-extrude-link.png|800px]]

Your wheel is now ready, and it comes with a default collision shape that is
a copy of the generated 3D mesh.

[[file:files/ftu5-custom-wheel.png|800px]]

Next, select `Exit Model Editor` from the `File` menu. Gazebo will prompt you
 to save the new model to disk. Press the "Save and Exit" button on the Exit
 dialog, and the "Save Model" dialog will appear.

[[file:files/ftu5-save-model.png|800px]]

Set the name of the new model to "HollowWheel", and fill the information under
 the `Advanced Options` section. Press the `Save` button. Your new Gazebo model  is ready to roll.

### Add the wheel to your Car



==================================
=================================

give info on downloading Inkscape
draw custom wheel, with suggestion dimensions (use M2 MOOC content)
show how to save properly
insert into Gazebo; explain each parameter
delete old wheels from robot and attach new ones
use align and snap tools again
save and place in scene

