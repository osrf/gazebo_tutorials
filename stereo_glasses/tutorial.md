# Introduction

This tutorial describes how to setup a machine and Gazebo for use with stero glasses, such as those provided by [Nvidia 3D Vision](http://www.nvidia.com/object/3d-vision-main.html). 

This tutorial assumes that you will be using Ubuntu.

## Hardware requirements

1. A [3D ready monitor](http://www.amazon.com/s/ref=nb_sb_noss_2?url=search-alias%3Daps&field-keywords=3D+monitor&rh=i%3Aaps%2Ck%3A3D+monitor). This monitor should have a 120Hz refresh rate.

1. An [Nvida Quadro K4000](http://www.amazon.com/s/ref=nb_sb_noss_2?url=search-alias%3Daps&field-keywords=quadro+k4200&rh=i%3Aaps%2Ck%3Aquadro+k4000) or better.
    **Warning: Many Nvidia cards say they support 3D stereo. Such a statement may only be true when using Windows drivers. Make sure there is a 3pin VESA connector on the card, otherwise the card will not work in Linux.**

1. A complete [Nvidia 3D Vision kit](http://www.amazon.com/s/ref=nb_sb_noss_1?url=search-alias%3Daps&field-keywords=nvidia+3d+vision+2): glasses, emitter, sync cable, and usb cable.

### Connections

1. Make sure you are using either a display port or dual-dvi cable. This is needed to support the high refresh rates.

1. Make sure the IR emitter is connected via both the USB and 3-pin VESA cables.

1. Make sure the glasses are charged.

## Sofware requirements

1. Ubuntu 14.04 (Trusty).

1. Install `libogre3d-1.9-dev`.

    ~~~
    sudo apt-get install libogre3d-1.9-dev
    ~~~

1. Install nvidia settings.

    ~~~
    sudo apt-get install nvidia-settings
    ~~~

1. Gazebo [compiled from source](http://gazebosim.org/tutorials?tut=install_from_source&cat=install). You may use the Gazebo6 debains when they are made available on July 27, 2015.

## Setup Nvidia Stereo

1. Start `nvidia-settings` manager.

    ~~~
    nvidia-settings
    ~~~

1. Set the refresh rate to 120Hz. Do not use `auto`.

1. Set the resolution. Do not use `auto`.

1. Select the `Save to X Configuration File` and follow instructions to save to `/etx/X11/xorg.conf`.

1. Add stereo option to `xorg.conf`. 

    ~~~
    sudo cp /etc/X11/xorg.conf /etc/X11/xorg.conf.original
    cd /tmp
    nvidia-xconfig -c xorg.conf -o xorg.conf --stereo=10
    sudo cp xorg.conf /etc/X11/xorg.conf
    ~~~

    **Note: The `--stereo=10` is for NVIDIA 3D VISION. Use 11 for NVIDIA 3D VISION PRO. For other settings see `man nvidia-settings`.**

1. At this point your `xorg.conf` file should look similar to:

    ~~~
    Section "ServerLayout"
        Identifier     "Layout0"
        Screen      0  "Screen0" 0 0
        InputDevice    "Keyboard0" "CoreKeyboard"
        InputDevice    "Mouse0" "CorePointer"
    EndSection
    
    Section "InputDevice"
        Identifier     "Mouse0"
        Driver         "mouse"
        Option         "Protocol" "auto"
        Option         "Device" "/dev/psaux"
        Option         "Emulate3Buttons" "no"
        Option         "ZAxisMapping" "4 5"
    EndSection
    
    Section "InputDevice"
        Identifier     "Keyboard0"
        Driver         "kbd"
    EndSection
    
    Section "Monitor"
        Identifier     "Monitor0"
        VendorName     "Unknown"
        ModelName      "Ancor Communications Inc VG248"
        HorizSync       30.0 - 160.0
        VertRefresh     50.0 - 150.0
        Option         "DPMS"
    EndSection
    
    Section "Device"
        Identifier     "Device0"
        Driver         "nvidia"
        VendorName     "NVIDIA Corporation"
        BoardName      "Quadro K4000"
    EndSection
    
    Section "Screen"
        Identifier     "Screen0"
        Device         "Device0"
        Monitor        "Monitor0"
        DefaultDepth    24
        Option         "metamodes" "nvidia-auto-select +0+0; 1920x1080_100 +0+0"
        Option         "SLI" "Off"
        Option         "MultiGPU" "Off"
        Option         "BaseMosaic" "off"
        Option         "Stereo" "10"
        SubSection     "Display"
            Depth       24
        EndSubSection
    EndSection
    ~~~

1. Restart X by logging out and logging back in.

## Verify Stereo

1. The Nvidia logo on the emitter should glow green. If red, then go back through the previous steps to make sure everything was done correctly.

1. Open `nvidia-settings`.

    ~~~
    nvidia-settings
    ~~~

1. Click on `X Screen 0`.

1. The `Stereo Mode` setting should say `NVIDIA 3D Vision Stereo`.

1. Install `mesa-utils`.

    ~~~
    sudo apt-get install mesa-utils
    ~~~

1. Run `glxgears` in stereo.

    ~~~
    glxgears -stereo
    ~~~

1. You should see blurry gears that look 3D when you put on the glasses.

## Setup Gazebo

1. Edit `~/.gazebo/gui.ini`.

    ~~~
    gedit ~/.gazebo/gui.ini
    ~~~

1. Add the following lines, if not already present.

    ~~~
    [rendering]
    stereo=1
    ~~~

    **Note: You can disable stereo by using `stereo=0`, or removing the `stereo` line.**

1. Run Gazebo.

    ~~~
    gazebo worlds/shapes.world
    ~~~
