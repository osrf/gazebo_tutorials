# Overview
This document provides instructions for replicating the setup used by participating HAPTIX teams.

Please note that you can install the full simulation environment on a standalone Linux machine by simply following the [installation tutorial](http://gazebosim.org/tutorials?tut=haptix_install&cat=haptix). This tutorial is for users who want to integrate 3D vision and motion capture for a full virtual reality experience.

# Inventory

- 1 64-bit computer with [3D Vision-compatible graphics card](http://www.nvidia.com/object/quadro_pro_graphics_boards_linux.html) and Ubuntu 14.04 (Trusty Tahr)
- 1 DisplayPort cable
- 1 DVI cable (for 2-computer setup)
- 1 Logitech keyboard
- 1 Logitech mouse
- 1 [3D Monitor](http://www.nvidia.com/object/3d-vision-displays.html)
- Nvidia 3D Vision 2 kit:
  - 1 pair 3D glasses
  - 1 infrared emitter
  - 1 3-pin VESA cable
  - 1 mini-USB cable
  - 1 micro-USB cable
- 1 3DConnexion Spacenav
- 1 USB switch with 2-port USB hub (for 2-computer setup)

# Hardware setup
There are two options to replicate the HAPTIX setup. One requires two physical computers, and the other requires one Linux computer (with a virtual machine installed).

## Two-computer setup

The two-computer HAPTIX system consists of both Linux computer and a Windows computer. Gazebo relies on Linux, while OptiTrack must run on Windows. A USB switch allows these two computers to share a single monitor, keyboard, and other peripherals. The following diagram depicts the connections between each component in the system.

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

For a one-computer setup, one Linux computer is required. A Windows virtual machine must be installed on the Linux computer to interface with the Optitrack. (You can learn more about virtual machines [here](http://www.howtogeek.com/196060/beginner-geek-how-to-create-and-use-virtual-machines/).)

1. Connect the Linux computer to power.

1. Connect the monitor to the computer via DisplayPort.

1. Connect all of the USB peripherals (keyboard, mouse, Spacenav, OptiTrack, emitter, and glasses if they need to be charged) to the Linux computer.

1. Connect the emitter to the 3-pin port on the back of the computer with the VESA cable.

1. Power the computer on.

1. Make sure to connect your computer to the internet with an Ethernet cable before continuing.

# Software setup

## Gazebo installation
To install the full simulation environment, open a Terminal (under "Applications", "Accessories") and type:

~~~
sudo apt-get install handsim
~~~

If this command does not work, you need to tell your computer it's okay to accept software from OSRF:

~~~
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-latest.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -;
sudo apt-get update
sudo apt-get install handsim
~~~

You can start Gazebo in the terminal by typing:

~~~
gazebo --verbose worlds/arat.world
~~~

## Desktop Environment (optional)

The HAPTIX teams were provided with a set of user-friendly icons and startup scripts. To replicate this setup, install the `haptix-tools` package with the following instructions.

WARNING: The `haptix-tools` package may be harmful to an existing desktop environment. In particular, if you have heavily modified your `xorg.conf`, you may want to avoid installing `haptix-tools`.

1. Create a new user called "haptix" by going to "System Tools", "Administration" and "User Accounts".

1. Open a terminal and type:

    ~~~
    sudo adduser haptix sudo
    ~~~
    
    to add the new user to the sudoers list. Reboot the machine.

1. Log back in as the new `haptix` user, open a terminal and type:

    ~~~
    sudo apt-get install haptix-tools
    ~~~

On the desktop, you should see three icons.

[[file:files/desktop.png]]

1. haptixStart: Double-click to launch the Gazebo prosthetic arm simulator.

1. haptixUpdate: Double-click to check for software updates and install them if found.

1. haptixSupport: This will open up a secure connection with the support staff at OSRF for troubleshooting purposes. If your simulation system or connection to peripheral hardware appears broken, please contact haptix-support@osrfoundation.org and schedule a support session. Then double-click on this icon to initiate the connection.

    If you want to reproduce the haptixSupport command, see the [OSRF wiki](http://wiki.osrfoundation.org/RequestingRemoteControl), and read more about [x11vnc](http://www.karlrunge.com/x11vnc/).

1. checkStereo: This will open a window that displays two rotating gears in stereo. Use this icon to test whether stereo is enabled on your system.

You can also make your own icons via the [Terminal](http://askubuntu.com/questions/457371/how-to-add-an-application-icons-to-the-desktop-in-14-04) or [graphically](http://askubuntu.com/questions/450266/an-easy-way-to-create-a-desktop-shortcut).

## Virtual machine installation (for one computer setup)
On your Linux machine, follow the link on [this website](http://www.vmware.com/products/player/playerpro-evaluation.html) to install VMWare player for Linux 64-bit.

Download the Windows virtual machine image from the OSRF web servers from [here](https://s3.amazonaws.com/osrf-distributions/haptix/vm/vm.tgz). WARNING: this is a large file, about 5 gigabytes.

Untar the file to `~/vmware`. The tarball should contain a file ending in `.vmx`. Start VMware Player using the graphical menu icon or in the terminal by typing `vmware-player`. Open `.vmx` file in VMWare Player. This should start the Windows virtual machine.

## Nvidia drivers
To find the correct Nvidia drivers for stereo vision, go to
[this page](http://www.nvidia.com/Download/index.aspx) and use the drop-down menus
to select your video card model. For the `Operating System` field, be sure to select "Linux 64-bit".

On the next page, note the number in the "Version:" field, but do not download anything.
Ignore the part of the version number after the dot.
For example, if the Nvidia website said your required driver version was 346.59, the version
number is simply "346", ignore the ".59" part.

If the version number is between 304 and 331:

~~~
sudo apt-get install nvidia-<version number>
~~~

If the number is greater than 331, you will need to install nvidia drivers from a PPA. Type the following into the terminal:

~~~
sudo add-apt-repository -y ppa:xorg-edgers/ppa
sudo apt-get update
~~~

Then:

~~~
sudo apt-get install nvidia-<version number>
~~~

You will need to restart your computer for the new drivers to take effect.


# Testing your setup

1. Make sure the stereo glasses are charged and turned on and the emitter is glowing dull green.

1. Double click on the Gazebo icon. You should see the Gazebo window come up, displaying the simulated prosthetic arm and manipulation environment. You should also see the stereo emitter change to bright green, indicating that stereo has been turned out.

1. The screen should appear "blurry", as if there are two images on top of each other.

1. Put the glasses on. The Gazebo window should look 3D.

1. You can use the keyboard to move the arm and the mouse to change the viewpoint. Or, you can use the Spacenav to control the arm and viewpoint position. Press the button on the Spacenav to toggle between arm and viewpoint. The number keys (1-5) will control pre-defined grasps (see the [teleop tutorial](http://gazebosim.org/tutorials?cat=haptix&tut=haptix_teleop) for more information).

1. Move to the [next tutorial](http://gazebosim.org/tutorials?cat=haptix&tut=haptix_optitrack) for instructions on configuring the OptiTrack 3D camera for viewpoint and arm pose control.

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
