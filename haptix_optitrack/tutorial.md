# Overview
This tutorial describes how to configure the Optitrack motion tracking system for use with the HAPTIX Gazebo simulator.

## What you'll need
All of the necessary items, including all software components, will be provided to participating HAPTIX teams. We are also providing links for any participants or hobbyists who want to create their own kit.

1. A Linux computer with the HAPTIX Gazebo simulator installed (see [here](http://gazebosim.org/tutorials?cat=haptix&tut=haptix_install) for instructions).

2. A Windows computer or a Windows virtual machine with the Motive motion tracking software installed. (Motive software installation instructions are [here](https://www.naturalpoint.com/optitrack/downloads/motive.html))

3. The Optitrack V120 Trio 3D camera and accompanying wires, hubs, etc. ([purchase here](https://www.naturalpoint.com/optitrack/products/v120-trio/))

4. A camera tripod, 1-2 meters high

5. A set of motion tracking targets (DIY participants can find motion tracking accessories here [here](https://www.naturalpoint.com/optitrack/products/suits-markers/); we recommend getting 9 reflective markers, 9 short bases, and 2 rigid body bases)

## Camera setup
Mount the Optitrack on the tripod.

The Optitrack kit comes with a long USB cable, a power cable, and a hub with several ports. Plug the power cable into the wall and into the hub. Then plug the USB cable into the hub. Later, you will use this cable to connect to the Windows computer. Finally, plug the wire coming out of the hub into the back of the Optitrack. Consult the [V120 Trio Quick Start Guide](https://www.naturalpoint.com/optitrack/static/documents/V120-Trio%20Quick%20Start%20Guide.pdf) if you need more help setting up the camera.

Place the Optitrack 1-2 meters to the right of the workstation desk, so that the camera is nearly rotated 90 degrees from the plane of the monitor and facing the workstation/the chair where the user will sit. The camera should be looking at the user's right side, with a slight over-the-shoulder perspective.  The cable connecting the Optitrack to the Windows computer is more than long enough, but take care not to catch it on anything and leave enough slack so that it doesn't pull.

Note: if you need to simulate the left arm, the physical setup must be flipped: place the camera on the left side of the workstation, and make sure all motion tracking targets are worn on the left side.

## Motion tracking target setup

### Arm and head trackers
Participating HAPTIX teams are provided with pre-made motion tracking targets that can be worn on the arm and head:

[[file:files/premade_trackers.png]]

If you already have arm and head trackers, skip to the next section on monitor tracker placement.

To create your own motion tracking targets, you will need at least 3 silver motion tracking spheres per target.

In the HAPTIX project, the Optitrack is used to track the pose of the user's arm, which directly controls the pose of the simulated robot arm. Additionally, the pose of the user's head is tracked to control the viewpoint in simulation. Therefore, you will need to create one motion tracking target that can be strapped to the user's arm, and another that can be worn on the head.

In a pinch, tracking targets can be made by taping or gluing motion tracking spheres to a piece of cardboard or rigid plastic:

[[file:files/diy_target.png]]

This target can then be affixed to a cloth or velcro armband to make an arm tracker.

To make a head tracker, affix the target to a headband. Alternatively, if you are using Nvidia 3D glasses, affix them to the right leg of the glasses.

### Monitor tracker placement
Participating HAPTIX teams are provided with three motion tracking spheres attached to round bases with double-sided tape on one side:

[[file:files/spheres.png]]

These spheres will be attached to the corners of the monitor associated with the Gazebo machine. Adhere one to the top right corner of the monitor, one to the top left corner, and another to the middle of the right side.

[[file:files/monitor.png]]

## Configuring the Motive Project File
If you received a premade set of tracking targets, download [this Motive Project file](https://bitbucket.org/osrf/gazebo_tutorials/raw/default/haptix_optitrack/files/haptix_osrf.ttp). We are going to make a small modification to it.

### Tracking rigid bodies
On your Windows machine, make sure the Optitrack is plugged in via USB, and start the Motive software by clicking on the shortcut on your desktop.

[[file:files/motive_icon.PNG]]

You should see the following screen pop up:

[[file:files/motive_start.PNG|800px]]

If you downloaded the premade project file, select "Open Existing Project" and then browse to and select the .ttp file that you downloaded. You can also open Motive with the existing project file just by double-clicking on the project file.

If you did not download the project file, close the startup menu.

If it isn't already visible, open the Rigid Body Properties view under "Views" in the top toolbar.

In the Camera Preview view, you should see three images with small white dots. The dots represent the position of motion tracker spheres in each camera frame (the Optitrack Trio has 3 cameras). In the 3D view, you should see the position of each sphere in 3D space.

[[file:files/camera_preview.PNG|800px]]

If you see dots or big circles that don't correspond to your motion tracking objects, there may be infrared interference in the scene. Try removing or hiding metallic/shiny objects and blocking nearby sources of natural light.

In Perspective View, figure out which spheres represent the spheres placed on your monitor. You can make it easier by taking the head and arm tracker outside of the view. Highlight those three markers by clicking and dragging the selection rectangle across them.

[[file:files/selection.PNG|800px]]

Click on "Create From Selection" in Rigid Body Properties. A triangle connecting your selected points and a label should appear on the 3D view:

[[file:files/new_rigid_body.PNG|800px]]

Rename the new rigid body "MonitorTracker". You can do this in Rigid Body Properties, or under the Project tab by right-clicking on the rigid body and selecting "Rename Asset".

Now, open "Coordinate System Tools" under "Tools" in the top toolbar. Highlight the three spheres on the monitor again in Perspective View, then click on "Set Ground Plane". You should see the monitor rigid body line up with the axes in Perspective View.

[[file:files/groundplane.PNG]]

If you created your own arm and head trackers:

Make sure the spheres from the previous rigid body are deselected. Hold up the arm target and select the spheres in the arm target by clicking and dragging. Hold the arm tracker band in the orientation that it will be worn. Extend out your arm straight, facing the monitor. Carefully move your arm close to the screen and try to align the target so that it is as perpendicular to the monitor as possible:

[[file:files/arm_alignment.png]]

Hold your arm still and click on "Create From Selection", as above. Rename the new rigid body "ArmTracker".

Similarly for the head target, select the associated spheres, making sure you are only selecting the head tracker spheres.

If you are using Nvidia 3D glasses, hold the glasses up as straight as possible like this, trying to align the long axis of the glasses with the long edge of the monitor:

[[file:files/head_alignment.png]]

If you made your own head tracking, put the head tracker on and hold your head still and as straight as possible, facing the monitor. I recommend making sure that the target is placed on a consistent position  (e.g. above the ear) when it is used.

Hold the tracker as still as possible and click on "Create from Selection". Rename the rigid body "HeadTracker".

Important: make sure to get the rigid body names correct, otherwise Gazebo will not associate the motion tracking targets to the corresponding object in simulation!

You should be able to move around the arm and head trackers and watch the positions change in the 3D view in Motive!

### Network configuration
If you did not start from the premade project file, you'll need to configure the network options so that the Windows Optitrack machine can pass information to the Linux Gazebo machine about rigid bodies.

To do this, open "Data Streaming" under "View" in the top toolbar and check the box next to "Broadcast Frame Data".

[[file:files/data_streaming.PNG]]

### Check the result
You're now done configuring Motive for your physical setup.  This is a good time to make sure that all the markers are being tracked correctly.  Bring the arm and head trackers back into the scene and verify that you see them move in the Camera and Perspective Views in Motive.

### Save the result
Select "Save Project" under "File" in the top toolbar (you can save the file under a different name or in a different location if you like; just remember what you called it and where you put it).

Whenever you want to run an experiment, repeat the initial steps of this tutorial to open Motive and select the .ttp file that you created here.  You should not need to modify the configuration in the future unless you change the marker placements.

## Starting Gazebo
Make sure the Optitrack is plugged in and then start Gazebo on the Linux machine:

~~~
gazebo --verbose worlds/arat.world
~~~

Put on the arm tracker and head tracker. Hold up your arm like the robot arm in simulation, and hold your head upright and straight. Then using your other hand, press spacebar to unpause simulation. Wave your arm around to control the simulated arm.

## How it works
Why is the monitor motion tracking target needed? The monitor acts as a reference frame for the arm and head trackers. If the camera is moved at any point, the settings in the Motive configuration file don't need to be recalibrated, because the pose data coming into Gazebo is calculated relative to the position of the monitor.

Why do the head and arm trackers need to be calibrated in such a specific way? The orientation of rigid bodies that Motive sends to Gazebo is measured relative to the initial position of the tracker. It is not relative to the camera frame. Therefore, the head and arm need to be calibrated relative to a known object. The coordinate axes of the monitor can be thought of as the coordinate axes of Gazebo, so the monitor frame is known. Therefore, we configure the head and arm trackers to be measured relative to the monitor.
