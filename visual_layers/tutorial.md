# Overview

From Gazebo 6, it is possible to add meta data to the visuals in your simulation. This tutorial explains how to add layer meta data to visuals so you can control which layers are visible via the graphical interface.

# Assigning layers on SDF

Currently, layers are identified by numbers. At your model SDF file, under each `<visual>` tag, you can add a `<meta>` tag for meta information and then a `<layer>` tag with the layer number as follows:

    <visual name='visual_0'>
      <meta>
        <layer>0</layer>
      </meta>
      (...)
    </visual>

Visuals without a layer assigned can't have their visibility toggled and will always be visible.

# Visualizing layers

An example [world](https://bitbucket.org/osrf/gazebo/src/gazebo6/worlds/shapes_layers.world) is distributed with Gazebo. You can load this world using the following command:

    gazebo worlds/shapes_layers.world

You can toggle the visibility of each layer via the `Layers` tab on the left panel:

[[file:files/layers_1.png|640px]]

[[file:files/layers_2.png|640px]]

If no visuals on the simulation have a layer, the layers tab will be empty.

