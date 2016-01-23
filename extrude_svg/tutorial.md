# Overview

This tutorial describes the process of extruding SVG files, which are 2D
 images, to create 3D meshes for your robots in Gazebo. It is sometimes
 easier to design part of a model in a program like [Inkscape](https://inkscape.org/) or [Illustrator](www.adobe.com/Illustrator).

Before starting, make sure you're familiar with the
 [Model Editor](http://gazebosim.org/tutorials?tut=model_editor).

This tutorial will show you how to make a custom wheel as an .svg in Inkscape,
 import it into Gazebo, so that it can be attached to a robot.

### Using the Inkscape SVG editor

[[file:files/inkscape_logo.jpg]]

There are many SVG editors. For this tutorial, we will use the Open Source
 Inkscape program (see
[installation instructions](https://inkscape.org/en/download) ).

 This is the wheel
[SVG file](https://bitbucket.org/osrf/gazebo_tutorials/raw/default/extrude_svg/files/wheel.svg) used in this tutorial.


[[file:files/wheel.svg|200px]]


### Document preparation

Start Inkscape. This will create a blank document. First, lets change the
 document size to better accommodate our wheel: under the `File->Document
 properties menu`, select the `Page` tab and change the document size to a
 custom size of 100.0 x 100.0 mm.

[[file:files/inkscape-page-tab.png|400px]]

Then, in the same dialog, select the `Grids`
 tab, press the `New` button to create a custom grid. Then, check the `Enabled`,
 `Visible`, and `Snap to visible grid lines only` options. Change
 `Spacing X` and `Spacing Y` to be 10.

[[file:files/inkscape-grids-tab.png|400px]]

You should end up with a document looking like this:

[[file:files/inkscape-blank.png|400px]]

### Draw

You can use the different tools (pen, text, stars and shapes, etc...) to create
 your geometry. In this example, the wheel is made from circles (pressing the
 Shift Key you can start your circle from the center, and using the CTRL key
 allows you to keep the roundness of the shape). It is possible to combine
 shapes together, making sure that paths are closed and that the part has
 a proper thickness.

 **Note**: a stick figure or two circles that  touch each other would not
 result in valid Gazebo models.

[[file:files/inkscape-simple-wheel.png|400px]]

Gazebo only imports `paths`, but it easy with Inkscape to transform any shape
 to a path. Select `Select All` from the `Edit` menu. Then select
`Path -> Object to Path` menu item. This will transform every object into
 separate paths and sub paths. This transformation is irreversible, so if you
 transform text into paths, you will not be able to alter the text.

[[file:files/inkscape-select-all.png|400px]]

Gazebo does not support grouping. Use the `Ungroup` from the `Object` menu to
 separate groups of paths.

### Save your drawing

Save your drawing to an SVG file you can use later in Gazebo. Use the `Save`
 menu from the `File` menu.

### Create a Gazebo Model

[SDFormat](http://sdformat.org) does not support SVG directly, it supports 2D
 poly lines. The Gazebo Model Editor has an import mechanism that extract the
 poly lines from SVG files, and save it them an SDF model file.

Launch Gazebo and Select `Model Editor` from the `Edit` menu to enter the
 Gazebo Model Editor mode (as opposed to the simulation mode).


Then press the `Add` button on the `Custom Shapes` from the `Insert` tab.

[[file:files/add-custom-shape.png|400px]]

This will open the `Import Link` dialog from where you can select the SVG file
 by pressing the `Browse` button.

[[file:files/import-link.png|400px]]

Once the file has been selected, press the `Import` button to open the
 `Extrude Link` dialog.

[[file:files/extrude-link.png|400px]]

The dialog allows you to set parameters of the extrusions:


* **Thickness**: How thick the link will be. This corresponds to the extrusion
 height in the `z` axis. For the SVG path shown on the right, the axis of
 extrusion is outwards from the screen.

* **Resolution**: How many pixels in your SVG correspond to a meter. The
 default value (3543.3 px/m) corresponds to 90 dpi (dots per inch), which is
 the default resolution for several editors, including Inkscape. If your model
 shows up the size you'd like in Inkscape when you display the units as meters,
 you shouldn't change the resolution value.

* **Samples per segment**: This indicates how many segments to divide each of
 the curved paths in the SVG. The more segments, the more complex your link
 will be. It doesn't change anything for straight paths.

On the right, you can see the path extracted from your SVG. The red points are
 the summit of the triangulation of the extruded 3D model.

Set the thickness of the wheel to 0.025 m, and press `OK`. Your new link should
 appear in the 3D view.

A new link is created, and it comes with a default collision shape that is
a copy of the generated 3D mesh.

[[file:files/custom-wheel.png|400px]]

Next, select `Exit Model Editor` from the `File` menu. Gazebo will prompt you
 to save the new model to disk. Press the `Save and Exit` button on the Exit
 dialog, and the `Save Model` dialog will appear.

[[file:files/save-model.png|400px]]

Set the name of the new model to "HollowWheel", and fill the information under
 the `Advanced Options` section. Press the `Save` button.

> Your new Gazebo model is now ready to roll ;-)


