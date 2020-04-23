# Introduction

It is often useful in simulation to have a ground plane with satellite imagery for applications that take place in large outdoor environments, such as aerial robotics. Satellite imagery is also useful for environment visualization and computer vision tasks.

This tutorial shows how to use the Static Map world plugin, supplied by Gazebo, to create and insert a satellite map model into a world. The images are downloaded at run time using the [Google's Static Map API](https://developers.google.com/maps/documentation/static-maps/intro) so internet connectivity is required to use this plugin.

# Example World

To try out this plugin, download and save [this world file](https://github.com/osrf/gazebo/raw/gazebo9/worlds/static_map_plugin.world) as `static_map_plugin.world`:

<include lang='xml' src='https://github.com/osrf/gazebo/raw/gazebo9/worlds/static_map_plugin.world'/>

The example world contains only a sun and the Static Map plugin. A `ground_plane` model is not needed as the plugin will be generating and inserting the map model into the world at run time. Before launching this world, you will need an API key. Since we are using the Google Static Map API service, you can get yourself a [Google API key](https://developers.google.com/maps/documentation/static-maps/get-api-key).

Once you have the API key, enter it into the plugin parameter in the world file, i.e. replace the following line with the key given to you by Google:

    <api_key>enter_your_google_api_key_here</api_key>

Once that is done, launch gazebo with the `static_map_plugin.world`.

    gazebo --verbose static_map_plugin.world

and you should see the following:

[[file:files/static_map_plugin.jpg|600px]]

In this example world, the plugin will generate a map model with a unique name and save the model SDF and image files in to a self-contained model folder with `map_` prefix in `<HOME>/.gazebo/models`. We are also telling the plugin that we don't want to use the image cache by setting `<use_cache>` to false so that subsequent launches will trigger the download of images (and recreate the model files) again. See the Plugin Parameters section below on how to specify you own model name or reuse image cache to prevent future image downloads.

Now try inserting a few models from the gazebo model database into the world (Left: original Google map view. Right Gazebo window):

[[file:files/static_map_models.png|800px]]


# Plugin Parameters

The plugin requires the following parameters:

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

* It is only able to generate square-shaped map models based on `<world_size>` and does not support rectangular regions.
* Downloading of images happen in the main thread and thus blocks Gazebo window until all images are downloaded. The number of image tiles to download depends on the `<world_size>` specified.
* Only the **Standard** Google Map API is supported, see [Usage Limits](https://developers.google.com/maps/documentation/static-maps/usage-limits) for more details.
