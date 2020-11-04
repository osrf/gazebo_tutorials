# Overview
This tutorial describes how to configure the Optitrack motion tracking system for use with the HAPTIX Gazebo simulator. You will learn about each piece of equipment and how to calibrate the markers so that each component works properly.

## What you'll need
All of the necessary items, including all software components, will be provided to participating HAPTIX teams. We are also providing links for any participants or hobbyists who want to create their own kit.

1. A Linux computer with the HAPTIX Gazebo simulator installed (see [here](http://gazebosim.org/tutorials?cat=haptix&tut=haptix_install) for instructions).

1. A Windows computer or a Windows virtual machine (VM) with the Motive motion tracking software installed. HAPTIX team machines come pre-installed with a Windows VM that initializes on startup. (Motive software installation instructions are [here](https://www.naturalpoint.com/optitrack/downloads/motive.html)).

1. The Optitrack V120 Trio 3D camera and accompanying wires, hubs, etc. ([purchase here](https://www.naturalpoint.com/optitrack/products/v120-trio/)).

1. A camera tripod, 1-2 meters high.

1. A set of motion tracking targets (DIY participants can find motion tracking accessories [here](https://www.naturalpoint.com/optitrack/products/suits-markers/); we recommend getting 9 reflective markers, 9 short bases, and 2 rigid body bases).

## Camera setup
Mount the Optitrack on the tripod.

The Optitrack kit comes with a long USB cable, a power cable, and a hub with several ports. Plug the power cable into the wall and into the hub. Then plug the USB cable into the hub. Later, you will use this cable to connect to the machine running Motive (or any other tracking program). Finally, plug the wire coming out of the hub into the back of the Optitrack. Consult the [V120 Trio Quick Start Guide](https://www.naturalpoint.com/optitrack/static/documents/V120-Trio%20Quick%20Start%20Guide.pdf) if you need more help setting up the camera.

Place the Optitrack 1-2 meters to the right of the workstation desk, so that the camera is nearly rotated 90 degrees from the plane of the monitor and facing the workstation/the chair where the user will sit. The camera should be looking at the user's right side, with a slight over-the-shoulder perspective.  The cable connecting the Optitrack to the Windows computer is more than long enough, but take care not to catch it on anything and leave enough slack so that it doesn't pull.

Note: if you need to simulate the left arm, the physical setup must be flipped: place the camera on the left side of the workstation, and make sure all motion tracking targets are worn on the left side.

[[file:files/fullsetup.png]]

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
If you received a premade set of tracking targets, download the Motive project file, [haptix_osrf.ttp](https://github.com/osrf/gazebo_tutorials/raw/master/haptix_optitrack/files/haptix_osrf.ttp) to the Desktop of the Windows machine. We are going to make a small modification to calibrate each component.

### Tracking rigid bodies
Make sure Gazebo is closed on Linux. On your Windows machine, make sure the Optitrack is plugged in via USB, and start the Motive software by clicking on the shortcut on your desktop.

[[file:files/motive_icon.PNG]]

You should see the following screen pop up:

[[file:files/motive_start.PNG|800px]]

If Motive displays the error message "License not found", the Optitrack was not plugged in correctly. Review the Camera Setup section above and double-check that the Optitrack is connected to power and to your computer via USB. If you are using a Windows virtual machine, make sure the "Natural Point Optitrack" USB device is turned on at the bottom right corner of the VM window.

Close the startup menu.

Select "Data Streaming" under "View" in the top toolbar and check the box next to "Broadcast Frame Data".

[[file:files/data_streaming.PNG]]

Now, select "File", "Open", and select the project file you downloaded, [haptix_osrf.ttp](https://github.com/osrf/gazebo_tutorials/raw/master/haptix_optitrack/files/haptix_osrf.ttp).

**Known bug with Motive: While running Motive on the Virtual Machine, make sure to enable "Broadcast Frame Data" before opening a project file! If you try to open the project file directly, Motive will crash.**

If it isn't already visible, open the Rigid Body Properties view under "Views" in the top toolbar.

In the Camera Preview view, you should see three images with small white dots. The dots represent the position of motion tracker spheres in each camera frame (the Optitrack Trio has 3 cameras). In the 3D view, you should see the position of each sphere in 3D space.

[[file:files/camera_preview.PNG|800px]]

If you see dots or big circles that don't correspond to your motion tracking objects, there may be infrared interference in the scene. Try removing or hiding metallic/shiny objects and blocking nearby sources of natural light.

In Perspective View, figure out which spheres represent the spheres placed on your monitor. You can make it easier by taking the head and arm tracker outside of the view. Highlight those three markers by clicking and dragging the selection rectangle across them.

[[file:files/selection.PNG|800px]]

Click on "Create From Selection" in Rigid Body Properties. A triangle connecting your selected points and a label should appear on the 3D view:

[[file:files/new_rigid_body.PNG|800px]]

Rename the new rigid body "MonitorTracker". You can do this in Rigid Body Properties, or under the Project tab by right-clicking on the rigid body and selecting "Rename Asset".

If you created your own arm and head trackers:

Make sure the spheres from the previous rigid body are deselected. Hold up the arm target and select the spheres in the arm target by clicking and dragging. Put on the arm tracker and hold your arm straight out, with your palm facing the ground. Make sure the tracker is aligned so that the sphere attached perpendicular to the marker is facing straight up, and the two diagonal prongs are facing towards the camera. (See the picture below in "Starting Gazebo" for arm posture reference.)

Hold your arm still and click on "Create From Selection", as above. Rename the new rigid body "ArmTracker".

Similarly for the head target, select the associated spheres, making sure you are only selecting the head tracker spheres.

If you are using Nvidia 3D glasses, put the head tracker on and hold your head still and as straight as possible, facing the monitor. Make sure that the target is placed in a position consistent with where it will be worn (e.g. above the ear).

Hold the tracker as still as possible and click on "Create from Selection". Rename the rigid body "HeadTracker".

**Important: make sure the rigid body names are correct, otherwise Gazebo will not associate the motion tracking targets to the corresponding object in simulation!**

You should be able to move around the arm and head trackers and watch the positions change in the 3D view in Motive!

### Check the result
You're now done configuring Motive for your physical setup.  This is a good time to make sure that all the markers are being tracked correctly.  Bring the arm and head trackers back into the scene and verify that you see them move in the Camera and Perspective Views in Motive.

### Save the result
Select "Save Project" under "File" in the top toolbar. Make sure it is saved to the Desktop as `haptix_osrf.ttp`.

You should not need to modify the configuration in the future unless you change the placement of the individual markers on a tracker or unless you move the angle of the camera significantly.

## Starting Gazebo
Make sure the Optitrack is plugged in and then start Gazebo on the Linux machine
by double-clicking on the `haptixStart` desktop icon.

Put on the arm tracker and head tracker. Hold up your arm like the robot arm in simulation, and hold your head upright and straight. Then using your other hand, press `v`, `b` or `n` to unpause simulation. Wave your arm around to control the simulated arm.

[[file:files/armcloseup.png]]
