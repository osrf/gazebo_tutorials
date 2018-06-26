# Flashlight Plugin Introduction

FlashLightPlugin is a [model plugin](/tutorials?tut=plugins_model&cat=write_plugin) included with gazebo that flashes and dims lights on a model. By giving parameters to the plugin, you can choose which lights to blink and also specify the duration and interval time of flashing for each of lights. By inheriting this plugin, you can also use internal features, e.g., dynamically turning the lights on/off.

# Usage and Plugin Parameters
Under `<model>` element, insert <plugin> element with `filename` attribute which is set to `libFlashLightPlugin.so`.

```XML
a
```

# How do they do it?
<!-- Details (Setting class and Topic) -->

<!-- diagram showing classes -->

## FlashLightSetting class

## ~/light/modify topic


# Example



# Extension of Plugin

<!-- diagram of extended plugin class -->


## Turning Lights On/Off

## Changing Duration/Interval

# Extension of Setting class

<!-- diagram of extended plugin class + setting class -->

## Instantiation and Initialization

## Flash/Dim
<!-- Flash/Dim -->

## Possible Scenario
