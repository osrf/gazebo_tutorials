# Introduction
This tutorial shows how to create a video from a camera in a gazebo world.

# Save Camera Images
Gazebo can automatically save camera images to disk.
To do so, a `<save>` tag must be added to a camera sensor.

## Create a world with a camera
Download and save [this world.](http://bitbucket.org/osrf/gazebo_tutorials/raw/default/camera_save/files/camera_tutorial.world)

<include from='/#include/' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/camera_save/files/camera_tutorial.world' />

```
<sensor name="my_camera">
  <camera>
    ...
    <save enabled="true">
      <path>/tmp/camera_save_tutorial</path>
    </save>
    ...
  </camera>
</sensor>
```

The `<save>` tag has an attribute `enabled` that must be set to `true` for images to be saved.
The child tag `<path>` is a directory in which the camera images will be saved.
If the directory does not exist, gazebo will try to create it.

## Run the world
Navige to the folder where the world was downloaded and start gazebo.

`gazebo --verbose camera_tutorial.world`

Close gazebo after a few seconds.

The world above will save images to `/tmp/camera_save_tutorial`.
It has a camera configured to capure images with a 1920x1080 resolution at 30 frames per second.

## Examine the images
Inside `/tmp/camera_save_tutorials` there should be many jpg images numbered sequentialy.
Each one has the same resolution as the camera they were captured from (1920x1080).
The first image is called `default_camera_link_my_camera(1)-0000.jpg`.

# Convert Images to Video
Now that the camera images are saved to disk, they can be converted to a video with [ffmpeg](https://ffmpeg.org/ffmpeg.html).
This comand creates a video called `my_camera.mp4` at 30 frames per second.

```
ffmpeg -r 30 -pattern_type glob -i '/tmp/camera_save_tutorial/default_camera_link_my_camera*.jpg' -c:v libx264 my_camera.mp4
```

[[file:files/my_camera.gif|480px]]

Download a video created from this tutorial [here.](http://bitbucket.org/osrf/gazebo_tutorials/raw/default/camera_save/files/my_camera.mp4)
