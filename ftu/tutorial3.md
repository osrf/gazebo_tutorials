# User Interface

This is an introduction to the Gazebo Graphical User Interface, or GUI. We will learn interface basics like what the buttons do and how to navigate in the scene. 

Start by double-clicking on the Gazebo icon to open Gazebo. 

## GUI

This is what you should see:

[[file:files/ftu3-boot-screen.png|600px]]

Note that the Gazebo interface consists of multiple sections, explained below. 

### The Scene

The Scene is the main part of the simulator. This is where the simulated
objects are animated and you interact with the environment. 

[[file:files/ftu3-scene.png|600px]]

### The Panels

Both side panels—right and left—can be displayed, hidden or resized by dragging
the bar that separates them from the scene.

[[file:files/ftu3-panels.png|600px]]

#### Left Panel

The left panel appears by default when you launch Gazebo. There are three tabs
in the panel:

* **WORLD** tab: The World tab displays the models that are currently in the
  scene, and allows you to view and modify the models' parameters, like their
  poses. You can also change the camera view angle by expanding the
  "GUI" option and tweaking the camera pose. 

* **INSERT** tab: The Insert tab is where you add new objects (models) to the
  simulation. Click (and release) on the model you want to insert, and click
  again in the Scene to add it.


* **LAYERS** tab: The Layers tab organizes and displays the different
  visualization groups that are available in the simulation, if any. A layer
  may contain one or more models. Toggling a layer on or off will display
  or hide the models in that layer. 
  
      This is an optional feature, so this tab will be empty in
      most cases. To learn more about Layers, check out the 
      [Visibility Layers](http://gazebosim.org/tutorials?tut=visual_layers&cat=build_robot) 
      tutorial. 

#### Right Panel (hidden by default)

The right panel is hidden by default. Click and drag the bar to open it. The right panel can 
be used to interact with the mobile parts of a selected model (the joints). If there are no 
models selected in the Scene, the panel does not display any information.

### The Toolbars

The Gazebo interface has two Toolbars. One is located just above the Scene, and
the other is just below.

#### Upper Toolbar

The main Toolbar includes some of the most-used options for interacting with
the simulator, such as buttons to: select, move, rotate, and scale objects;
create simple shapes (e.g. cube, sphere, cylinder); and copy/paste. Go ahead and play 
around with each button to see how it behaves.

[[file:files/ftu3-top-toolbar.png|800px]]

#### Bottom Toolbar

The Bottom Toolbar displays data about the simulation, like the simulation time
and its relationship to real-life time. "Simulation time" refers to how quickly
time is passing in the simulator when a simulation is running.  Simulation can
be slower or faster than real time, depending on how much computation is
required to run the simulation. 

"Real time" refers to the actual time that is passing in real life as the
simulator runs. The relationship between the simulation time and real time is
known as the "real time factor" (RTF). It's the ratio of simulation time to real
time. The RTF is a measure of how fast or slow your simulation is running
compared to real time. 

[[file:files/ftu3-bottom-toolbar.png|800px]]

### The Menu 

Like most applications, Gazebo has an application menu up top. Some of the menu
options are duplicated in the Toolbars or as right-click context menu options
in the Scene. Check out the various menus to familiarize yourself. 

[[file:files/ftu3-gazebo-menu.png|800px]]

NOTE: Some Linux desktops hide application menus. If you don't see the
menus, move your cursor to the top of the application window, and the menus
should appear. 

[[file:files/ftu3-gazebo-menu.png|800px]]

## Mouse Controls

The mouse is very useful when navigating in the Scene. We highly recommend
using a mouse with a scroll wheel.  Below are the basic mouse operations for
navigating in the Scene and changing the view angle.

Right-clicking on models will open a context menu with various options.
Right-click on a model now to see what's available. 


[[file:files/ftu3-mouse-controls.png|600px]]
