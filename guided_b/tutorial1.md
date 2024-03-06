# Intro

Welcome to the Beginner Module! This module will guide you through the most
basic features of Gazebo. We will build a simple vehicle to demonstrate these
features. 

Each tutorial builds upon the last, so we recommend following the tutorials in order. 

These tutorials are intended for those new to Gazebo and/or folks with **no programming or Linux experience**.

# What is Gazebo? #

Gazebo is a 3D dynamic simulator with the ability to accurately and
efficiently simulate populations of robots in complex indoor and outdoor
environments. While similar to game engines, Gazebo offers physics
simulation at a much higher degree of fidelity, a suite of sensors, and
interfaces for both users and programs.

Typical uses of Gazebo include:

* testing robotics algorithms,

* designing robots,

* performing regression testing with realistic scenarios

A few key features of Gazebo include: 

* multiple physics engines,

* a rich library of robot models and environments,
 
* a wide variety of sensors,

* convenient programmatic and graphical interfaces

# System requirements #

Gazebo is currently best used on [Ubuntu](http://www.ubuntu.com/download), a flavor of [Linux](https://en.wikipedia.org/wiki/Linux). You will need a computer that has:

* A dedicated [GPU](https://en.wikipedia.org/wiki/Graphics_processing_unit),
 * Nvidia cards tend to work well in Ubuntu
* A CPU that is at least an Intel I5, or equivalent,
* At least 500MB of free disk space,
* Ubuntu Trusty or later installed.

# Installation Instructions for Ubuntu #

If you run into problems during the install, please see the [complete install instructions](/tutorials?tut=install_ubuntu&cat=install).

1. Download the installer. Right-clicking the [link to the installer](http://get.gazebosim.org) and choose 
"Save Link As" to the Downloads directory of your disc.

1. Copy the following text by pressing Ctrl-C:

    ```
    chmod +x ~/Downloads/gazebo.sh
    ```

1. Press Alt-F2
  * A window with a prompt should appear in the upper left
  

1. Press Ctrl-V to paste the text, and press Enter
  * The window will disappear
  
 
1. Copy the following by pressing Ctrl-C:

    ```
    gnome-terminal --working-directory="~" -e "./Downloads/gazebo.sh"
    ```

1. Press Alt-F2
  * A window with a prompt should appear in the upper left
  

1. Press Ctrl-V to paste the text, and press Enter
  * A new window will appear with a password prompt.
  

1. Enter your password, and press Enter

1. If prompted, press Y to continue

1. Wait until the window disappears

# Run Gazebo #

1. Press Alt-F2
2. Type "gazebo", and press Enter.
