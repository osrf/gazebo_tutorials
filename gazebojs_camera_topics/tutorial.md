
# Save camera images as jpeg files

In this example, we will describe how to save a number of successive camera images to jpeg files. This is done using subscribeToImageTopic, a specialized version of the subscribe function that allows you to save images in popular formats.
This method only works with 'gazebo.msgs.ImageStamped' topics, and accepts the following options:
- format: either jpeg (default) or png
- encoding: either binary (default) or base64

Png files are uncompressed (whereas jpeg are smaller). If you want to save the image to disk, use the binary encoding. The base64 encoding is useful if you want to send the image data to another client in a portable way (ex: to make a mjpg stream for web browsers).

### Code

Create a file

    gedit save_jpeg.js

and add the following content (https://bitbucket.org/osrf/gazebojs/raw/default/examples/save_jpeg.js):

<include src='https://bitbucket.org/osrf/gazebojs/raw/default/examples/save_jpeg.js' />


### Code explained


First, we load the necessary scripts and modules into the script engine.

~~~
var util = require('util');
var gazebojs = require('gazebojs');
var fs = require('fs');
~~~

Then we add the basic functionality: a function to generate padded numbers (ie "007" instead of "7") and we collect arguments:

- which camera to get images from (we then compute the full topic name: "~/camera/link/camera/image"

- how to name the saved images ( "frame\_" to get names like "frame\_000.jpeg")

- how many images to save before exiting the program.

~~~

// adds 0 in front a string for padding
function pad(num, size) {
    var s = num+"";
    while (s.length < size) s = "0" + s;
    return s;
}

if (process.argv.length != 5)
{
  console.log( 'node ' + process.argv[2] + ' [source camera name] [dest_path] [count]');
  process.exit(-1);
}

var gazebo = new gazebojs.Gazebo();
var src_camera = process.argv[2];
var dest_path = process.argv[3];
var framesToSave = parseInt(process.argv[4]);

var src_topic  = '~/' + src_camera + '/link/camera/image';

var savedFrames = 0;

console.log('saving [' + src_topic + '] to  [' + dest_path + '] for ' + framesToSave + ' frames');

~~~

Then we call the subscribeToImageTopic and provide a callback method. In this case, we use jpeg images with binary encoding so we can save the files to disk.

~~~

options = {format:'jpeg', encoding:'binary' }

gazebo.subscribeToImageTopic(src_topic, function (err, img){
        savedFrames += 1;
        if(err) {
            console.log('error: ' + err);
            return;
        }

        if (savedFrames > framesToSave) {
            console.log('bye');
            process.exit(0);
        }
        
        // make a nice zero padded number (0003 instead of 3)
        var nb = pad(savedFrames, 4);
        var fname = dest_path + '_' + nb + '.jpeg' ;
        fs.writeFile(fname, img, {encoding:'binary'}, function (err) {
            if(err) 
                console.log('ERROR: ' + err);
            else
                console.log(fname + ' saved');
         });
        
    }, options);

~~~

Because of the asynchronous nature of our script, we setup callbacks to keep the script alive until all the images have been processed. The Gazebo object is referenced to avoid premature garbage collection on older versions of NodeJs.

~~~

console.log('setup a loop with 5 sec interval tick');
setInterval(function (){
  console.log('tick ' + gazebo);
},5000);

~~~

### Testing the code

First, you must setup Gazebo. In an empty world, add a few items (the double pendulum is a good one because it is animated) and drop a camera. Make sure you get the name of your camera (the default name for the first camera is "camera").


[[file:files/world.png|640px]]


Invoke the script

    node save_jpeg.js camera frame 10
    
You should see the following output:

~~~
node save_jpeg.js camera frame 10
saving [~/camera/link/camera/image] to  [frame] for 10 frames
setup a loop with 5 sec interval tick
frame_0001.jpeg saved
frame_0002.jpeg saved
frame_0003.jpeg saved
frame_0004.jpeg saved
frame_0005.jpeg saved
frame_0006.jpeg saved
frame_0007.jpeg saved
frame_0008.jpeg saved
frame_0009.jpeg saved
frame_0010.jpeg saved
bye
~~~

Inspect your images.

[[file:files/frame_0007.png|640px]]

