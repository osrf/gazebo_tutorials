# Introduction
This tutorial shows how to create a video from a camera sensor in a gazebo world.

# Save Camera Images
Gazebo can automatically save camera images to disk.
To do so, a `<save>` tag must be added to a camera sensor.

## Create a world with a camera
Download and save [this world.](http://bitbucket.org/osrf/gazebo_tutorials/raw/default/camera_save/files/camera_tutorial.world) as `camera_tutorial.world`

<include from='/#include/' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/camera_save/files/camera_tutorial.world' />


The important parts are here.

%%%
<sensor name="my_camera">
  <camera>
    <image>
      <width>1920</width>
      <height>1080</height>
    </image>
    ...
    <save enabled="true">
      <path>/tmp/camera_save_tutorial</path>
    </save>
    ...
    <update_rate>30</update_rate>
  </camera>
</sensor>
%%%

The `<save>` tag has an attribute `enabled` that must be set to `true` for images to be saved.
The child tag `<path>` is a directory in which the camera images will be saved.
If the directory does not exist, gazebo will try to create it.
`<width>` and `<height>` set the resolution of the images.
`update_rate` is the number of images per second that will be saved.
This camera will output images with 1920x1080 resolution at 30 frames per second.

## Run the world
Navigate to the folder where the world was downloaded and start gazebo.

`gazebo --verbose camera_tutorial.world`

Close gazebo after a few seconds.

The world above will save images to `/tmp/camera_save_tutorial`.

## Examine the images
Inside `/tmp/camera_save_tutorials` there should be many images numbered sequentialy.
They have the same resolution as the camera they were captured from (1920x1080).
The first image is called `default_camera_link_my_camera(1)-0000.jpg`.

# Convert Images to Video
Now that the camera images are saved to disk, they can be converted to a video with [ffmpeg](https://ffmpeg.org/ffmpeg.html).
This comand creates a video called `my_camera.mp4` at 30 frames per second.

```
ffmpeg -r 30 -pattern_type glob -i '/tmp/camera_save_tutorial/default_camera_link_my_camera*.jpg' -c:v libx264 my_camera.mp4
```

If you have Ubuntu Trusty or a newer version, you might have `avconv` instead of `ffmpeg`
([here is some of the backstory](https://en.wikipedia.org/wiki/Libav#Fork_from_FFmpeg)).
The following command can be used with `avconv`:

```
avconv -framerate 30 -i /tmp/camera_save_tutorial/default_camera_link_my_camera\(1\)-%04d.jpg -c:v libx264 my_camera.mp4
```

[[file:files/my_camera.gif|480px]]

Download the video [here.](http://bitbucket.org/osrf/gazebo_tutorials/raw/default/camera_save/files/my_camera.mp4)
