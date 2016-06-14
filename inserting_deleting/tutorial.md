There are many ways to insert and delete entities during a Gazebo simulation.
You can choose a method according to your use case:

#### A. Using the graphical interface (recommended for beginners)

#### B. From the command line (for those comfortable with the terminal)

#### C. From a C++ program (either a plugin or a standalone program)

#### D. Using Javascript and NodeJS ??

# A. Graphical interface

To use the graphical interface, open a terminal and start Gazebo:

~~~
gazebo
~~~

### Insert a simple shape

Gazebo provides 3 simple shapes on the top toolbar, i.e. box, cylinder and
sphere. Click on one of them and then on the 3D scene to insert it.

[[file:files/simple_shapes_toolbar.png]]

### Insert a simple light

Gazebo provides 3 light types on the top toolbar, i.e. point light, spot light
and directional light. Click on one of them and then on the 3D scene to insert
it.

[[file:files/lights_toolbar.png]]

### Insert a pre-built model

Does the model database get loaded automatically for new users?

[[file:files/insert_tab.png]]

### Delete a model

There are various ways:

* Right-click the 3D scene, `Delete` (this doesn't work for lights)

    [[file:files/model_context_menu.png]]

* Right-click the model list, `Delete`

    [[file:files/model_list_context_menu.png]]

* Select model and press `Del` key

# B. Command line tool

It's possible to insert and delete models from simulation using the `gz model`
tool.

### Delete a model

1. Start your simulation in Gazebo. For example, start a world with simple shapes:

    ~~~
    gazebo worlds/shapes.world
    ~~~

1. Open a new terminal and type the following to delete the box:

    ~~~
    gz model --delete -m "box"
    ~~~

1. The box will disappear in simulation.

### Insert models

1. Start your simulation in Gazebo. For example, start an empty world:

    ~~~
    gazebo worlds/empty.world
    ~~~

1. Open a new terminal.

1. To insert a model from an SDF file saved in your computer, use the absolute
file path and give a unique name for the model in simulation, for example:

    ~~~
    gz model -m "cabinet" --spawn-file "/home/<user>/.gazebo/models/cabinet/model.sdf"
    ~~~

    > Tip: You can find installed models in `/home/<user>/.gazebo/models`.

1. The model will appear in simulation.

> Tip: It's also possible to spawn by passing a string:

~~~
gz model -m "my_model" --spawn-string | ?
~~~

> Tip: You can specify the pose where the model will appear, for example:

~~~
gz model -m "drill" --spawn-file "/home/<user>/.gazebo/models/cordless_drill/model.sdf" -x 0 -y 0 -z 5 -R 0 -P 1 -Y 0
~~~

# C. Programmatically

Gazebo's transport library provides a simple API to request entity deletion and
insertion.

### Request deletion

1.



### What's happening in the back?

If you're interested in learning what the Gazebo API is doing,
this section is for you.

The server provides the ignition service `<world_name>/request` to receive
requests such as insertion and deletion of entities. This service receives
`ignition::msgs::Operation` messages.

Gazebo's transport API

> Tip: You can learn more about ignition transport
> [here](http://ignition-transport.readthedocs.io/en/latest/)



* Operation message to `/request` service. Explain message, especially all the factory options.

* Use `transport::Request`

* No direct methods? You can pause simulation and send the request?

### Standalone program example






