# Introduction

In applications such as aerial robotics, it is often useful to have a ground plane with satellite imagery in simulation to help with testing computer visual tasks. The tutorials shows how to use the Static Map world plugin supplied by gazebo to create and insert a flat satellite map model into the world. The images are downloaded in run time using the [Google's Static Map API](https://developers.google.com/maps/documentation/static-maps/intro) so internet connectivity is required to use this plugin.

# Example World

To try out this plugin, download and save [this world file](https://bitbucket.org/osrf/gazebo/raw/462957509d71f7bf1dd0f981950a53a097cba9ae/worlds/static_map_plugin.world) as `static_map_plugin.world`:

<include lang='xml' src='https://bitbucket.org/osrf/gazebo/raw/462957509d71f7bf1dd0f981950a53a097cba9ae/worlds/static_map_plugin.world'/>

Since we are using the Google Static Map API service, you'll need to get yourself a [Google API key](https://developers.google.com/maps/documentation/static-maps/get-api-key)

Once you have the API key, enter it into the plugin parameter in the world file, i.e. replace the following line with key generate given to you by Google

    <api_key>enter_your_google_api_key_here</api_key>

Once that is done, launch gazebo with the `static_map_plugin.world`.

    gazebo --verbose static_map_plugin.world

and you should see the following:

[[file:files/static_map_plugin.jpg|600px]]


Try inserting a few models into the world (Left: original google map view. Right Gazebo window):

[[file:files/static_map_models.png|600px]]


# Plugin Parameters

The plugin has the following required parameters:

* `<center>`: Latitude and longitude of center of map. The two values should be space delimited.
* `<world_size>`: Target size of world to cover (in meters). The plugin will fetch enough tiles to create a model that is larger than the specified size.
* `<api_key>`: [Google API key](https://developers.google.com/maps/documentation/static-maps/get-api-key).

Optional parameters:

* `<model_name>`: Name of map model. If this is not specified, the plugin will try to generate a unique name based on the plugin parameters.
* `<pose>`: Pose of map model in the world.
* `<zoom>`: Zoom level from 0 (entire world) to 21 (streets) or higher. Note: Google does not have satellite images at a zoom level of 21+ in some regions in the world so you may have to lower the zoom level if images are not found.
* `<map_type>`: Type of map to use: `roadmap`, `satellite`, `terrain`, `hybrid`. The default is `satellite`.
* `<tile_size>`: Size of map tiles in pixels. Max of 640 for standard Google Static Map usage.
* `<use_cache>`: True to use the model found in `GAZEBO_MODEL_PATH` if it exists, otherwise recreate the model and save it in `<HOME>/.gazebo/models`

# Limitations

The Static Map Plugin currently has few limitations:

* It is only able to generate square-shaped map models and it does not support capturing a rectangular region of the world.
* Downloading of images happen in the main thread and thus blocks gazebo window until all images are downloaded. The number of image tiles to download depends on the `<world_size>` specified.
* Only the **Standard** Google Map API is supported, see [Usage Limits](https://developers.google.com/maps/documentation/static-maps/usage-limits) for more details.
