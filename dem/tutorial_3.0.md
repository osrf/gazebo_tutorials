# DEM Compatibility

Gazebo packages are not compiled with support for DEM. You will need to compile gazebo from source using the default branch after have installed the gdal packages (remember to run cmake after the packages installation) to get a version of gazebo with DEM support.

#Overview

A Digital Elevation Model (DEM) is a 3D representation of a terrain's surface that does not include any objects like buildings or vegetation. DEMs are frequently created by using a combination of sensors, such as LIDAR, radar, or cameras. The terrain elevations for ground positions are sampled at regularly-spaced horizontal intervals. [Wikipedia](http://en.wikipedia.org/wiki/Digital_elevation_model) is a good resource for getting more details about DEMs.

The term DEM is just a generic denomination,  not a specific format. In fact, the DEMs can be represented as a grid of elevations (raster) or as a vector-based triangular irregular network (TIN). Currently, Gazebo only supports raster data in the supported formats available in [GDAL](http://www.gdal.org/).

The main motivation to support DEMs in Gazebo is to be able to simulate a realistic terrain. Rescue or agriculture applications might be interested in testing their robot behaviors using a simulated terrain that matches the real world.

# Bring DEM support to Gazebo

In order to work with DEM files you should install GDAL libraries.

**On Ubuntu Trusty:**

~~~
$ sudo apt-get install gdal-bin libgdal-dev libgdal1h python-gdal
~~~

**On Ubuntu Precise:**

~~~
$ sudo apt-get install gdal-bin libgdal1-1.7.0 libgdal1-dev python-gdal
~~~

# DEM file and the definition into SDF format

There are several organizations that provide elevation data. As an example, let's download a DEM file of Mount St. Helens [before](http://extract.cr.usgs.gov/public/NED/mtsthelens_before.zip) or [after](http://extract.cr.usgs.gov/public/NED/mtsthelens_after.zip) its eruption back in the '80s. Unzip the file and rename it `mtsthelens.dem`.

~~~
$ cd ~/Downloads
$ wget http://extract.cr.usgs.gov/public/NED/mtsthelens_before.zip
$ unzip ~/Downloads/mtsthelens_before.zip -d /tmp
$ mv /tmp/30.1.1.1282760.dem /tmp/mtsthelens.dem
~~~

Usually, DEM files have big resolutions and Gazebo cannot handle it, so it's a good idea to adjust the resolution of your DEM. The next command will scale the terrain to 129x129 and will copy into the Gazebo `media/dem/` directory.

~~~
$ mkdir -p /tmp/media/dem/
$ gdalwarp -ts 129 129 /tmp/mtsthelens.dem /tmp/media/dem/mtsthelens_129.dem
~~~

A DEM file in Gazebo is loaded in the same way that you load a heightmap image. Gazebo automatically detects if the file is a plain image or a DEM file. Create the file `volcano.world` and copy the next content. Save the file anywhere you want, for example, in /tmp.

<include src='http://github.com/osrf/gazebo_tutorials/raw/default/dem/files/volcano.world' />

The `<heightmap><size>` element in the code above tells Gazebo whether to load the DEM with the original dimensions (when `<size>` is not present) or to scale it (when `<size>` is present). In case you prefer to scale the DEM, the `<size>` element tells Gazebo the size in meters that the terrain will have in the simulation. If you want to maintain the correct aspect ratio, be sure to properly calculate the width, height and elevation (which is the third number in `<size>`). In our example, the DEM will be scaled to a square of 150 x 150 meters and a max elevation of 50 meters.

Launch Gazebo with the world containing your DEM file and you should see the volcano. In our case, the file is in the /tmp directory.

~~~
# Be sure of sourcing gazebo setup.sh in your own installation path
$ GAZEBO_RESOURCE_PATH="$GAZEBO_RESOURCE_PATH:/tmp" gazebo /tmp/volcano.world
~~~

[[file:files/gazebo_sthelens.png|640px]]

# How do I get a DEM file of my region of interest?

Next, we are going to describe one method for obtaining a DEM file of a specific region of interest.

[Global Land Cover Facility](http://glcf.umd.edu/) maintains a high-resolution digital topographic database of Earth. Go to its [Search and Preview tool](http://glcfapp.glcf.umd.edu:8080/esdi/index.jsp) and you will see something similar to the image below. Every terrain patch has a unique path and row that you should know before using the tool. We'll use QGIS to discover the path/row of our region of interest.

[[file:files/glcf_search_tool.png|640px]]

[QGIS](http://www.qgis.org/) is a cross-platform open source geographic information system program that provides data viewing, editing, and analysis capabilities. Download QGIS following the [instructions detailed on the QGIS website](http://www.qgis.org/en/site/forusers/download.html).

Open up QGIS, click on the left column icon labeled `WMS/WMTS layer`, click on `Add default servers`, select `Lizardtech server`, and then, press the `connect` button. Select the `MODIS` value and press `Add`. Close the pop-up window. The next step is to add another layer with all the different patches available. Download [this shapefile](http://landsat.usgs.gov/documents/wrs2_descending.zip) and decompress it in any folder. Go back to QGIS and press `Add Vector Layer` (left column icon). Press `Browse`, and select your previously uncompressed wrs2_descending.shp file. Press OK in the window that opens. Now, you'll see both layers on the main window. Let's change the transparency of the wrs2_descending layer to be able to see both layers at the same time. Double click on wrs2_descending layer, and then, modify its transparency value to something around 85%.

[[file:files/qgis.png|640px]]

Use the scroll and left button to navigate to your region of interest. Then click on the icon labeled `Identify Features` on the top bar. Click on your region of interest and all the terrain patches around the area will be highlighted. A new pop up window will show the path/row values for each highlighted patch. In the image below you can see the path and row of the DEM patch containing Las Palmas, one of the heavenly places of the Canary Islands, Spain.

[[file:files/qgis_las_palmas.png|640px]]

Go back to your browser with the GLCF search tool and write the path/row values in the columns labeled `Start Path` and `Start Row`. Then click in `Submit Query`; press `Preview and Download` to see your results. Choose your terrain file and press `Download`. Finally, select your file with extension .gz, and decompress it in your favorite folder. Global Land Cover Facility files are in GeoTiff format, one of the most common format of DEM files available.

# Preparing DEM data for use in Gazebo

DEM data is usually created at very high resolution. Use *gdalwarp* to reduce the resolution of the terrain to a more manageable size before using it in Gazebo.

~~~
$ gdalwarp -ts <width> <height> <srcDEM> <targetDEM>
~~~

DEM data often contain "holes" or "void" areas. These sections correspond to areas where data could not be collected while the DEM was created. In the case of a data "hole", the hole will be assigned the minimum or maximum value of the data type that is used in that DEM.

Always try to download "finished" versions of DEM data sets, where the holes have been filled. If your DEM terrain contains holes (also known as NODATA values), try to manually repair it using gdal tools, such as *gdal_fillnodata.py*.

# Working with multiple DEMs in Gazebo

Although Gazebo does not directly support multiple DEMs, GDAL has a set of utilities for merging a set of DEMs into a single one. The first step is to download the set of DEMs that you want to merge. Note that the patches can even overlap with one another; GDAL will merge them seamlessly. Assuming that your current directory contains a set of Geotiff files ready to be merged, run the next command.

~~~
$ gdal_merge.py *.tif -o dem_merged.tif
~~~

Now, you can just use `dem_merged.tif` in your world file and Gazebo will load the terrain with all the patches merged. In the next screenshot you can see the result of merging four terrain patches surrounding the Canary Islands.

[[file:files/gazebo_dem_merged.png|640px]]
