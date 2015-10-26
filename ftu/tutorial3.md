# User Interface

This is an overview of the Gazebo Graphical User Interface. We're not going to
make our simple robot quite yet, but we'll become familiar with the tools we'll
need to do so in the next tutorial.

## GUI

This is the gazebo screen just after invoke the simulator:

# [[file:files/gazebo-boot-screen.png|600px]]

Note that gazebo is composed of several parts in this main screen. This
turorial will give a quick look into every of them:

### The Scene

The scene is the main part of the simulator where the simulated objects are
animated and the user can interact with the enviroment. It takes the central
place in the application GUI, between the toolbars and panels.

~~~
image
~~~

### The Panels

Both side panels, right and left, can be displayed, hidden or resized by
pulling the bar that separates them from the scene.

# [[file:files/gazebo-boot-screen-panels.png|600px]]

#### Left panel

The left panel appears by default when launching gazebo application and provides
the abilitiy to interact with the simulator in different ways from the tabs 
available:

* **WORLD** tab: The world tab allows the user to check out and modify a good
  part of the parameteres that affects your simulation. The whole list of the
  simulated objects (models) can be explored, the physics parameters can be
  adjusted or the user main view on the scene (meny entry GUI > Camera) can be
  moved.

* **INSERT** tab: The insert tab allows the user to interactively add new
  objects (models) to the simulation by drag and drop them into the simulation
  screen.

* **LAYERS** tab: The layers tab displays the different visualization groups
  that are available in the simulation, if any. This is an optional feature to
  help organizing the visuals of the simulation but the tab can be empty in most
  of the cases.

#### Right panel (hidden by default)

The right panel is hidden by default, pull from the bar to open it. The right
panel can be used to interact with some of the different mobile parts of a
robot (the joints). If there is no object or static objects in the simulator 
the panel does not display any information.

### The Toolbars

The gazebo screen present two different toolbars located just above and below
of the Scene part.

#### Upper toolbar

The main gazebo toolbar includes some of the most used actions when interacting
with the simulator such as: selection, move, rotate and scale objects; create 
simple static shapes or copy/paste.

~~~
image
~~~

#### Bottom bar

The bottom toolbar displays data about the simulation, specially the simulation
time and its relationship with the real life time. This is known as real time
factor.

~~~
image
~~~

### The Menu 

Like most of applications into the system gazebo has an application menu 
integrated into the windows system. Some of the menu options are also 
duplicated into the toolbars or right click context menu.

~~~
image
~~~

## Mouse Controls

• Scene navigation, basic controls (Undo), etc.


