
# Save camera images as jpeg files

In this example, we will describe how to save a number of successive camera images to jpeg files. This is done using subscribeToImageTopic, a specialized version of the subscribe function that allows you to save images in popular formats.
This method only works with 'gazebo.msgs.ImageStamped' topics, and accepts the following options:
- format: either jpeg (default) or png
- encoding: either binary (default) or base64

Png files are uncompressed (whereas jpeg are smaller). If you want to save the image to disk, use the binary encoding. The base64 encoding is useful if you want to send the image data to another client in a portable way (ex: to make a mjpg stream for web browsers).

### Code

Create a file

    gedit save_jpeg.js

and add the following content:

<include src='https://bitbucket.org/osrf/gazebojs/raw/default/examples/save_jpeg.js' />


### Code explained


First, we load the necessary scripts and modules into the script engine.

<include from='/var util = require/' to='/require(\'fs\')/' src='http://bitbucket.org/osrf/gazebojs/raw/default/examples/save_jpeg.js' />

~~~
var util = require('util');
var gazebojs = require('gazebojs');
var fs = require('fs');
~~~

=== 

