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

# Topics in Gazebo 7, 8, and 9+
This plugin uses two topics: `contain` and `enable`.
In gazebo 7 these topics are gazebo topics using the message type `gazebo.msgs.Int`.
A value of `1` is considered `true` and a value of `0` is `false`.
Gazebo 8 has these gazebo topics, and adds duplicates using ignition transport.
The ignition transport topics use a message of type `ignition.msgs.Boolean`.
In gazebo 9 and beyond the gazebo topics are removed; only the ignition transport topics are available.

## Plugin parameters

### `<enabled>`
  If `true` the plugin will output data right away, otherwise it needs to be enabled explicitly.
  If unspecified this parameter defaults to `true`.
  Send a `true` value to the topic called `<namespace>/enable` to enable the plugin, and `false` to disable it.
  While disabled ContainPlugin will not publish any messages.

  *example: `<enabled>false</enabled>`*

### `<entity>`
  This is a scoped name of a link or model or nested model in the world.
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


# Example of Triggering an Action
This example shows how to use ContainPlugin to trigger an action.
When a ball rolls under a lamp post, the light will turn on.

[[file:example_world.png]]

Download the [example world](https://bitbucket.org/osrf/gazebo_tutorials/raw/default/contain_plugin/custom_example.world), [plugin source code](https://bitbucket.org/osrf/gazebo_tutorials/raw/default/contain_plugin/TurnOnLightPlugin.cpp), and [CMake file](https://bitbucket.org/osrf/gazebo_tutorials/raw/default/contain_plugin/CMakeLists.txt).
Build and run the plugin using [this tutorial](/tutorials?tut=plugins_hello_world&cat=write_plugin) as an example.

A ContainPlugin instance is configured to watch when a sphere enters a box below the lamp post.
When the sphere rolls into or out of the box ContainPlugin will publish a message on the `contain` topic.
<include lang='xml' from="/<plugin name='ContainPlugin'/" to="/    <\/pluign>/" src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/contain_plugin/contain_example.world'/>

This custom plugin receives messages from ContainPlugin, and turns the light on or off.
<include lang='c++' from="/#include/" to="/}  // namespace gazebo/" src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/contain_plugin/TurnOnLightPlugin.cpp'/>

After building and running the world you should see the ball trigger the light to turn on.
Hit `CTRL + R` to restart the world if you missed it.

[[file:lamppostlight.gif]]
