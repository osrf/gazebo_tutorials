# Overview

This document provides instructions for installing and configuring a HAPTIX
setup.

# Inventory

- 1 Zareason Linux computer with power cable
- 1 DisplayPort cable
- 1 DVI cable
- 1 Logitech keyboard
- 1 Logitech mouse
- 1 3D Monitor 
- Nvidia 3D Vision 2 kit:
  - 1 pair 3D glasses
  - 1 infrared emitter
  - 1 3-pin VESA cable
  - 1 mini-USB cable
  - 1 micro-USB cable
- 1 3DConnexion Spacenav
- 1 USB switch with 2-port USB hub

# Hardware setup
## Two-computer setup

The HAPTIX system consists of both a Linux and Windows computer. Gazebo relies on Linux, while OptiTrack must run on Windows. A USB switch allows these two computers to share a single monitor, keyboard, and other peripherals. The following diagram depicts the connections between each component in the system.

[[file:files/haptix_setup_diagram_final.svg]]

1. Connect both the Linux computer and the Windows computer to power. 

1. Connect the Linux computer to the monitor using the DisplayPort cable and the Windows computer with the DVI cable. 

1. Connect one Nvidia emitter to a USB port on the Linux computer.

1. Connect the VESA cable of the same emitter to the 3-pin DIN port on the back of the Linux computer.

1. If you are using stereo on the Windows computer: connect a second Nvidia emitter to the Windows computer.

1. The USB switch has two USB cables. Connect one to the Linux computer and one to the Windows computer.

1. Connect the keyboard and mouse to the USB switch.

1. Connect the 2-port hub to the USB switch. Optionally connect the 2-port hub to its power adaptor.

1. Connect the Spacenav to the 2-port hub.

1. Power on the computers.

1. You should see the Nvidia stereo emitter shine dull green when the computer it is connected to is powered on.

1. You may need to charge the Nvidia glasses before using them. Plug them into the computer using the micro-USB cable.

1. To switch between the two computers, press the button connected to the USB switch, and use the buttons on the monitor display to switch the input source (pictures).

## One computer setup

You can also take advantage of the Windows virtual machine installed on the
Linux computer in a one-computer setup. (You can learn more about virtual machines [here](http://www.howtogeek.com/196060/beginner-geek-how-to-create-and-use-virtual-machines/).)

1. Connect the Linux computer to power.

1. Connect the monitor to the computer via DisplayPort.

1. Connect all of the USB peripherals (keyboard, mouse, Spacenav, OptiTrack, emitter, and glasses if they need to be charged) to the Linux computer.

1. Connect the emitter to the 3-pin port on the back of the computer with the VESA cable.

1. Power the computer on.

1. Make sure to connect your computer to the internet with an Ethernet cable before continuing.

# Software setup

Log in using the provided user name and password.

## Change your password

For security reasons, you should change your password.

1. In the top toolbar, click on "Applications", then hover over "System Tools" and "Administration". Click on "User Accounts".

1. Click on "Unlock" and type your password to unlock administrator privileges. Click on "Password" to change your password.

You can add new user accounts if multiple people expect to use the machine, or you can simply share the password to one user account among multiple users. If you want to new user accounts to be able to add/remove/update software, make sure to make them Administrator accounts.

## Change your timezone (optional)

Change your timezone by clicking on the time in the upper-right hand corner, selecting "Time & Date settings", and clicking on the appropriate timezone.

# Environment overview
#
On the desktop, you should see three icons.

[[file:files/desktop.png]]

1. haptixStart: Double-click to launch the Gazebo prosthetic arm simulator.

1. haptixUpdate: Double-click to check for software updates and install them if found.

1. haptixSupport: This will open up a secure connection with the support staff at OSRF for troubleshooting purposes. If your simulation system or connection to peripheral hardware appears broken, please contact haptix-support@osrfoundation.org and schedule a support session. Then double-click on this icon to initiate the connection.

If you want to reproduce the virtualOSRF command, see the [OSRF wiki](http://wiki.osrfoundation.org/RequestingRemoteControl), and read more about [x11vnc](http://www.karlrunge.com/x11vnc/).

You can make your own icons via the [Terminal](http://askubuntu.com/questions/457371/how-to-add-an-application-icons-to-the-desktop-in-14-04) or [graphically](http://askubuntu.com/questions/450266/an-easy-way-to-create-a-desktop-shortcut).

# DIY machine configuration

If you did not receive a team machine but are configuring a Linux machine to use the HAPTIX simulation suite, you can install the icons as well as helpful start-up scripts and configuration tools.

1. Open a Terminal (under "Applications", "Accessories") and type:

    ~~~
    `sudo apt-get install haptix-tools`
    ~~~

    If this command does not work, you need to tell your computer it's okay to accept software from OSRF:
    
    ~~~
    sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-latest.list'
    wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -;
    sudo apt-get update
    ~~~
    
    Try the command in step 2 again.

1. Next, you can install the full simulation environment by typing:

    ~~~
    `sudo apt-get install handsim`
    ~~~

# Testing your setup

1. Make sure the stereo glasses are charged and turned on and the emitter is glowing dull green.

1. Double click on the Gazebo icon. You should see the Gazebo window come up, displaying the simulated prosthetic arm and manipulation environment. You should also see the stereo emitter change to bright green, indicating that stereo has been turned out.

1. The screen should appear "blurry", as if there are two images on top of each other.

1. Put the glasses on. The Gazebo window should look 3D.

1. You can use the keyboard to move the arm and the mouse to change the viewpoint. Or, you can use the Spacenav to control the arm and viewpoint position. Press the button on the Spacenav to toggle between arm and viewpoint. The number keys (1-5) will control pre-defined grasps (see the [teleop tutorial](http://gazebosim.org/tutorials?cat=haptix&tut=haptix_teleop) for more information).

1. Move to the [next tutorial](http://gazebosim.org/tutorials?cat=haptix&tut=haptix_optitrack) for configuring the OptiTrack 3D camera for viewpoint and arm pose control.

# Troubleshooting

1. If the screen does not look blurry and stereo does not work, contact OSRF support (haptix-support@osrfoundation.org).

1. Disconnecting and reconnecting the emitter will cause it to glow red and stop functioning. This is usually fixable by logging out and logging back in again. If at any point the emitter is red and logging off does not fix the emitter, please contact OSRF support.

1. If the Spacenav is non-functional, first check if the blue LED light is on. If it is on, open up a Terminal and type:

    ~~~
    sudo service spacenavd restart
    ~~~

    You will have to type in your password.

    If the Spacenav does not glow blue even if it is connected, or the terminal command does not work, contact OSRF support.

1. If Gazebo does not load or the arm is unresponsive to keyboard input, contact OSRF support.
