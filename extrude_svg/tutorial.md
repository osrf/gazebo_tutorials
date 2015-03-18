# Overview

This tutorial describes the process of extruding SVG files, which are 2D images, to create 3D meshes for your robots in Gazebo.

# Model Editor

Before starting, make sure you're familiar with the [Model Editor](http://gazebosim.org/tutorials?tut=model_editor).

# Prepare SVG files

Using a program such as [Inkscape](https://inkscape.org/), draw paths for each of the meshes you'll be extruding. As an example, here are two images which will be used to make a simple car: a [chassis](https://bitbucket.org/osrf/gazebo_tutorials/raw/default/extrude_svg/files/chassis.svg) and a [wheel](https://bitbucket.org/osrf/gazebo_tutorials/raw/default/extrude_svg/files/wheel.svg).

[[file:files/chassis.svg|400px]]

[[file:files/wheel.svg|400px]]

> **Tip**: If you're using Inkscape, display the units in meters so you have a feel of how big the parts will come up.

> **Note**: Currently Gazebo will take the center of the path's bounding box as the origin.

> **Note**: Currently Gazebo is not able to import very heavy files or files containing crossed paths.

> **Note**: Different SVG editors might have features which are not supported, such as special ready-made shapes and transformations.

> **Note**:

# Extruding the SVG

We will be using the Model Editor to make links out of extruded SVGs.

In the Model Editor, press the `Add` button under "Custom Shapes", you'll see the Import Link dialog.

[[file:files/import_link.svg|400px]]

Choose the `chassis.svg` file and hit Import. If you picked a valid SVG file, the Extrude Link dialog will come up.

[[file:files/extrude_link.svg|400px]]

* **Thickness**: How thick the link will be in the axis perpendicular tot he screen.

* **Resolution**: How many pixels in your SVG correspond to a meter. The default value (3543.3 px/m) corresponds to 90 dpi (dots per inch), which is the default resolution for several editors, including Inkscape. If your model shows up the size you'd like in Inkscape when you display the units as meters, you shouldn't change the resolution value.

* **Samples per segment**: This indicates how many segments to divide each of the curved paths in the SVG. The more segments, the heavier and more complex your link will be. It doesn't change anything for straight paths.

On the right, you can see the path extracted from your SVG. The chassis contains 12 points and is about 1m wide. The origin is indicated by the blue cross.

Press Ok to accept the settings, the extruded link will show up.

[[file:files/extruded_link.svg|400px]]

The link consists of a visual and a collision, both having the same geometry. A [polyline geometry](http://sdformat.org/spec?ver=1.5&elem=geometry#geometry_polyline) consists of a list of points and an extrusion height, so when your model is saved to SDF, the SVG file is not needed.

Now go ahead and extrude the wheel, in this example it was extruded to 0.2 m. Copy and paste it 3 times, position the wheels, add joints and your car is ready.

[[file:files/extruded_car.svg|400px]]
