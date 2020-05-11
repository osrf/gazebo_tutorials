# Contain Plugin Introduction

ContainPlugin is a [world plugin](/tutorials?tut=plugins_world&cat=write_plugin) included with gazebo that publishes a message when something enters or exits a volume.
It can be used to trigger an action.
For example, it may be used to raise an error in a test when a robot enters a keepout zone.

# Using in a world
This plugin is a world plugin, so it must be used as a child of `<world>`.
The `filename` attribute must be set to `libContainPlugin.so`.

```XML
<world name="default">
  <plugin name="MyContainPluginInstance" filename="libContainPlugin.so">
    <!-- plugin parameters go here ... -->
```

# Topics in Gazebo 9+
This plugin uses two ignition transport topics: `contain` and `enable`. The ignition transport topics use a message of type `ignition.msgs.Boolean`.

## Plugin parameters

### `<enabled>`
  If `true` the plugin will output data right away, otherwise it needs to be enabled explicitly.
  If unspecified this parameter defaults to `true`.
  Send a `true` value to the topic called `<namespace>/enable` to enable the plugin, and `false` to disable it.
  While disabled ContainPlugin will not publish any messages.

  *example: `<enabled>false</enabled>`*

### `<entity>`
  This is a scoped name of a link, model, nested model, or light in the world.
  Every simulation update ContainPlugin checks if the origin of this entity is inside the specified geometry.

  *example: `<entity>robot::hand_link</entity>`*

### `<namespace>`
  This is the prefix given to the topics used by ContainPlugin: `contain`, and `enable`.

  *example: `<namespace>/foo/bar</namespace>` would result in topics `/foo/bar/contain` and `/foo/bar/enable`.*

### `<pose>`
  This is the location of the center of the specified geometry.
  If the `frame` attribute is not specified then the pose is in world frame.
  If the `frame` attribute is set to a scoped name, then the pose is relative to the origin of that entity.

  *example: `<pose>1 2 3 0 0 0</pose>` in world frame*

  *example: `<pose frame="tree::limb">0.2 0 0.4 1.570769 0 0</pose>` relative to an entity*

### `<geometry>`
  This is the geometry which is checked to see if it contains `<entity>`.
  The only geometry currently supported by ContainPlugin is `<box>`.
  See the [sdformat specification for box geometry](http://sdformat.org/spec?ver=1.6&elem=geometry#geometry_box) for more info.


# Examples

## Using ContainPlugin to Trigger an Action
This example shows how to use ContainPlugin to trigger an action.
When a ball rolls under a lamp post, the light will turn on.

[[file:example_world.png|800px]]

Download the [example world](https://github.com/osrf/gazebo/raw/gazebo9/examples/plugins/trigger_light/contain_example.world), [plugin source code](https://github.com/osrf/gazebo/raw/gazebo9/examples/plugins/trigger_light/TurnOnLightPlugin.cpp), and [CMake file](https://github.com/osrf/gazebo/raw/gazebo9/examples/plugins/trigger_light/CMakeLists.txt).
Build and run the plugin using [this tutorial](/tutorials?tut=plugins_hello_world&cat=write_plugin) as an example.
Make sure to run `source <install_path>/share/gazebo/setup.sh` before extending `GAZEBO_PLUGIN_PATH`.
`<install_path>` is `/usr` if you installed using `apt`.

A ContainPlugin instance is configured to watch when a sphere enters a box below the lamp post.
When the sphere rolls into or out of the box ContainPlugin will publish a message on the `contain` topic.
<include lang='xml' from="/<plugin name='ContainPlugin'/" to="/    <\/pluign>/" src='https://github.com/osrf/gazebo/raw/gazebo9/examples/plugins/trigger_light/contain_example.world'/>

This custom plugin receives messages from ContainPlugin, and turns the light on or off.
<include lang='c++' from="/#include/" to="/}  // namespace gazebo/" src='https://github.com/osrf/gazebo/raw/gazebo9/examples/plugins/trigger_light/TurnOnLightPlugin.cpp'/>

After building and running the world you should see the ball trigger the light to turn on.
Hit `CTRL + R` to restart the world if you missed it.

[[file:lamppostlight.gif]]

## ContainPlugin tracking a Moving Volume
The `<pose>` tag on this plugin can be given relative to another entity.
This allows the volume to move with the entity as if it were connected by a fixed joint.
Gazebo includes a world demonstrating this.

Start gazebo in paused mode with the moving geometry world.

```
gazebo --pause --verbose worlds/contain_plugin_moving_demo.world
```

In another terminal echo the output of the `contain` topic,

```
ign topic --echo /gazebo/default/drill/contain
```

Initially the plugin reports a false value becaues the drill is not inside it.
As the volume falls the plugin reports a true value when the drill enters the volume, and a false value later when that is no longer the case.

[[file:movingvolume.gif]]
