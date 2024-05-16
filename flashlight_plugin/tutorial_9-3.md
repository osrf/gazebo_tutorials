# Introduction

[[file:files/example.gif|640px]]

FlashLightPlugin is a [model plugin](/tutorials?tut=plugins_model&cat=write_plugin) included with Gazebo-classic that flashes and dims light objects on a model. By giving parameters to the plugin, you can choose which lights to blink and also specify the duration and interval time of flashing for each of lights. By inheriting this plugin, you can also use internal features, e.g., turning the lights on/off.

# Usage and Plugin Parameters
Insert your plugin block with the `filename` attribute set to `libFlashLightPlugin.so` within the `<model>` element. In the following example (the world file is available [here](https://github.com/osrf/gazebo/raw/gazebo9/worlds/flash_light_plugin_demo.world)), the model has two links each of which has two light objects.

```XML
<model name='light_model'>

  ...

  <link name='cylinder'>

    ...

    <light name='light_source1' type='spot'>
      ...
    </light>

    <light name='light_source2' type='spot'>
      ...
    </light>

    ...

  </link>
  <link name='box'>

    ...

    <light name='light_source3' type='spot'>
      ...
    </light>

    <light name='light_source4' type='spot'>
      ...
    </light>

    ...

  </link>

  ...

  <plugin name='light_control' filename='libFlashLightPlugin.so'>
    <enable>true</enable>
    <light>
      <id>cylinder/light_source1</id>
      <duration>0.5</duration>
      <interval>0.5</interval>
    </light>
    <light>
      <id>cylinder/light_source2</id>
      <duration>0.3</duration>
      <interval>1.2</interval>
      <color>1 0 0</color>
    </light>
    <light>
      <id>box/light_source3</id>
      <duration>1.0</duration>
      <interval>0.1</interval>
      <enable>false</enable>
    </light>
    <light>
      <id>box/light_source4</id>
      <block>
        <duration>1.0</duration>
        <interval>0</interval>
        <color>1 1 0</color>
      </block>
      <block>
        <duration>1.0</duration>
        <interval>0.3</interval>
        <color>0 1 1</color>
      </block>
      <enable>true</enable>
    </light>
  </plugin>
</model>
```

The following items are the parameters which the plugin takes.
## `<light>`
This element represents a unit of settings for each flashlight. It can contain the following items. You can use this element as many times as the number of lights contained in your model.

## `<id>`
This element is required for `<light>`. It specifies which light you are going to control. It is composed of the link name followed by a slash "/" and the light name. In the example, you have a `<light>` named "light\_source1" under the `<link>` named "cylinder". So the `<id>` should be "cylinder/light_source1".

## `<duration>`
This element is required for `<light>`. It specifies for how long the light should be on in seconds.

## `<interval>`
This element is required for `<light>`. It specifies for how long the light should be dimmed in seconds. If it is set to 0, the light will be static.

## `<color>`
This element is optional for `<light>`. It specifies the color of the light. If it is not given, the default color of the visual object will be used. The format is RGB, each of which ranges from 0 to 1. For example, `1 0 0` represents red. `0.5 0.5 0.5` will be gray.

## `<block>`
This element is optional for `<light>`. It must have `<duration>` and `<interval>`, and it can optionally have `<color>`. `<light>` can have more than one `<block>` so it can produce multiple patterns with different colors. If this element is given, the `<duration>`, `<interval>`, and `<color>` elements directly placed under the `<light>` will be ignored.

For example,

```XML
<block>
  <duration>1</duration>
  <interval>0</interval>
  <color>1 0 0</color>
</block>
<block>
  <duration>1</duration>
  <interval>0</interval>
  <color>0 1 0</color>
</block>
<block>
  <duration>1</duration>
  <interval>0</interval>
  <color>0 0 1</color>
</block>
```

This setting will first provide 1 second of red light, followed by green and blue lights. After the blue light is casted, it goes back to the first one, i.e., red.

[[file:files/switchcolors.gif|640px]]

## `<enable>`
This element is optional. When it is given under the plugin, it specifies whether all the lights are enabled or not. If it is placed directly under an individual `<light>`, it overrides the global one and affects the corresponding light.

# Implementation Details
The diagram below shows an abstract structure of the plugin and its components. `FlashLightPlugin` class holds `FlashLightSetting` objects, each of which holds a unit of settings and maintains the corresponding light element by the Gazebo-classic transport topic.

[[file:files/flashlight.png|640px]]

## FlashLightSetting class
Once the plugin is loaded, it reads the parameters given under the `<plugin>` element. For each `<light>` element, an object of `FlashLightSetting` is created with the given parameters.

To flash/dim the light, `FlashLightSetting` class has two functions: `Flash()` and `Dim()`. It continuously checks the simulation time and finds the right timing to call those functions. Let's say the light to control is now flashing. When the duration time has been passed, it calls `Dim()`. Then, it waits until the interval time is passed. After that, it calls `Flash()`, and repeats these steps above. If the flashlight is given with multiple `<block>`, it switches to the next block so the light patterns are switched as described by the plugin parameters.

## ~/light/modify topic
Gazebo-classic advertises `~/light/modify` topic to update lights in the simulation. `Flash()` and `Dim()` store values in [msgs::Light](https://github.com/osrf/gazebo/blob/gazebo9/gazebo/msgs/light.proto) and send it to this topic so a light appearance reflects to the specified values. Particularly, `Flash()` sets `range` to a non-zero value, and `Dim()` sets it to 0.

# Extension of Plugin
FlashLightPlugin class has member functions which are accessible to derived classes. These functions can dynamically turn the flashlights on and off, and can also update the duration and interval time. As the diagram below shows, a derived plugin calls member functions of FLashLightPlugin to control the flashlights. The plugin could let external entities control flashlights by reacting to external events or requests.

[[file:files/extendedplugin.png|640px]]

## Turning Lights On/Off
A derived plugin can turn on/off a specific flashlight or all the existing lights on the model. If you want to access a particular one, you need to specify the light name and link name as function parameters. If an empty string is given to the link name, the function will access the first match of the light name.

## Changing Duration/Interval
The duration and interval time of flashing can be updated by calling the corresponding functions. The function parameter is the desired time to which the value is set.

# Extension of Setting class
You can also add functionalities at the exact timing when the light flashes and dims, by extending the `FlashLightSetting` class. This lets you synchronize other entities (such as visual objects) with the lights. [LedPlugin](/tutorials?tut=led_plugin&cat=plugins) is an example to blink visual objects at the same timing.
The figure below shows that a plugin now contains derived objects of FlashLightSetting.

[[file:files/extendedsetting.png|640px]]

## Flash/Dim
By overriding Flash and Dim functions, the inheriting setting class can do its job when the flashlight flashes and dims.

## Instantiation and Initialization
An extended setting class must be instantiated in the process shown in the figure blow. An extend plugin will need to override member functions of `FlashLightPlugin`.

[[file:files/init.png|640px]]

When a plugin is loaded, CreateSetting function is called to generate a setting object for each flashlight.

```C++
std::shared_ptr<FlashLightSetting>
  FlashLightPlugin::CreateSetting(
    const sdf::ElementPtr &_sdf,
    const physics::ModelPtr &_model,
    const common::Time &_currentTime)
{
  return std::make_shared<FlashLightSetting>(_sdf, _model, _currentTime);
}
```

This function must be overridden by the extended plugin so an object of the extended setting class can be generated.

```C++
std::shared_ptr<FlashLightSetting> ExtendedPlugin::CreateSetting(
  const sdf::ElementPtr &_sdf,
  const physics::ModelPtr &_model,
  const common::Time &_currentTime)
{
  return std::make_shared<ExtendedSetting>(_sdf, _model, _currentTime);
}
```

An object is initialized by InitSettingBySpecificData function. If the object is required to be initialized by data stored in the extended plugin, it must be done in an overridden InitSettingBySpecificData function, where the FlashLightSetting's InitSettingBySpecificData is also called.

```C++
void ExtendedPlugin::InitSettingBySpecificData(
    std::shared_ptr<FlashLightSetting> &_setting)
{
  // Call the function of the parent class.
  FlashLightPlugin::InitSettingBySpecificData(_setting);

  // Do something to initialize the object by the data in the extended plugin.
}
```
