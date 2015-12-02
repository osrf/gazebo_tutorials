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
 transform text into paths, you won't be able to alter the text.

[[file:files/ftu5-inkscape-select-all.png|800px]]

### Import

Gazebo does not support SVG directly, it only supports 2D polylines. That's OK
 because the Gazebo Model Editor has an import mechanism that extract the
 polylines from SVG files and provides a preview.



==================================
=================================

give info on downloading Inkscape
draw custom wheel, with suggestion dimensions (use M2 MOOC content)
show how to save properly
insert into Gazebo; explain each parameter
delete old wheels from robot and attach new ones
use align and snap tools again
save and place in scene

