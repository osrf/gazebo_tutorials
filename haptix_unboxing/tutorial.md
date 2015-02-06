## Overview

This documents gives instructions for installing and configuring your HAPTIX machine from OSRF.

## Inventory

- 1 Zareason computer with power cable and Display Port cable
- 1 Logitech keyboard
- 1 Logitech mouse
- Nvidia 3D Vision 2 kit:
  - 1 pair 3D glasses
  - 1 infrared emitter
  - 1 3-pin VESA cable
  - 1 mini-USB cable
  - 1 micro-USB cable
- 1 3DConnexion Spacenav
- switch (TODO: finalize)

## Hardware setup
Connect the computer to power. 

(Explain hooking up USB peripherals)

Power the computer on.

You should see the Nvidia Stereo emitter shine dull green upon power-up.

## Software setup

Log in using the provided username and password.

### Change your password
For security reasons, you should change your password. In the top toolbar, click on "Applications", then hover over "System Tools" and "Administration". Click on "User Accounts".

Click on "Unlock" and type your password to unlock adminstrator privileges. Click on "Password" to change your password.

You can add new user accounts if multiple people expect to use the machine, or you can simply share the password to one user account among multiple users. If you want to new user accounts to be able to add/remove/update software, make sure to make them Administrator accounts.

### Change your timezone
Optional: change your timezone by click on the time in the upper-right hand corner, selecting "Time & Date settings", and clicking on the appropriate timezone.

## Environment overview

On the desktop, you should see three icons.

[[file:files/desktop.png]]

Gazebo: Double-click to launch the Gazebo prosthetic arm simulator.

haptixUpdate: Double-click to check for software updates and install them if found.

VirtualOSRF: This will open up a secure connection with the support staff at OSRF for troubleshooting purposes. If your simulation system or connection to peripheral hardware appears broken, please contact haptix-support@osrfoundation.org and schedule a support session. Then double-click on this icon to initiate the connection.

## Testing your setup

Make sure the stereo glasses are charged and turned on (you should see a shining green LED indicator) and the emitter is glowing dull green.

Double click on the Gazebo icon. You should see the Gazebo window come up, displaying the simulated prosthetic arm and manipulation environment. You should also see the stereo emitter change to bright green, indicating that stereo has been turned out.

The screen should appear "blurry", as if there are two images on top of each other. If it is not, go to the left menu, click on "Scene", and check the box next to "Shadows".

Put the glasses on. The Gazebo window should look 3D.

You can use the keyboard to move the arm and the mouse to change the viewpoint. Or, you can use the Spacenav to control the arm and viewpoint position. Press the button on the Spacenav to toggle between arm and viewpoint. The number keys (1-5) will control pre-defined grasps (see the teleop tutorial for more information) ((link)).

Move to the next tutorial for configuring the Optitrack 3D camera for viewpoint and arm pose control (link).

## Troubleshooting

If clicking on the "Shadows" box does not enable stereo, contact OSRF support (haptix-support@osrfoundation.org).

Disconnecting and reconnecting the emitter will cause it to glow red and stop functioning. This is usually fixable by logging off and logging back in again. If at any point the emitter is red and logging off does not fix the emitter, please contact OSRF support.

If the Spacenav is non-functional, first check if the blue LED light is on. If it is on, open up a Terminal (under "Applications", "Accessories") and type:

~~~
sudo spacenavd
~~~

You will have to type in your password.

If the Spacenav does not glow blue even if it is connected, or the terminal command does not work, contact OSRF support.

If Gazebo does not load or the arm is unresponsive to keyboard input, contact OSRF support.
