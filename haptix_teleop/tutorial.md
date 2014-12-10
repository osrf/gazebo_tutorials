# Overview

This tutorial covers various teloperation methods for use in the HAPTIX simulation environment. The hardware devices supported include:

1. Keyboard: Control the arm position and joints using a set of keys

1. [Spacenav](http://www.amazon.com/s/ref=nb_sb_noss_1?url=search-alias%3Dmi&field-keywords=space+navigator&rh=n%3A11091801%2Ck%3Aspace+navigator): Control arm position using a 3D mouse

1. [K-Org mixer](http://www.amazon.com/Korg-nanoKONTROL-USB-Controller-White/dp/B001H2P294): Control joints using a midi mixer

1. [Razer hydra](http://www.ebay.com/sch/i.html?_from=R40&_trksid=p2050601.m570.l1313.TR3.TRC1.A0.H0.Xrazer+hydra&_nkw=razer+hydra&_sacat=0): Control arm position and grasps using a 3D gaming joystick.

## Keyboard Control

[[file:files/keyboard.png | 640px]]

A set of keys, as shown in the above figure, are mapped to arm and hand control.

1. **Start simulation**

    ~~~
    gazebo worlds/arat.world
    ~~~

1. **Grasp control**

    Press and hold a number key, between one and three to close the hand using:

       1. Spherical grasp
       2. Cylinderical grasp
       3. Pinch grasp

    To release the hand, press and hold SHIFT and the number key. 

1. **Arm position**

    Use the WASD keys to move the position of the arm in a 2D plane. The plane is defined by the local coordinate frame of the arm. This means the hand will "fly" like a plane, and is most noticable if you change the arm's orientation. 

        1. W: Move forward (along the positive X axis: toward the finger tips)
        2. A: Move left (along the positive Y axis: toward the thumb)
        3. S: Move back (along the negative X axis: toward the wrist)
        4. D: Move right (along the negative Y axis: opposite of the thumb)

    Use the QE keys to move up and down.

        1. Q: Move the arm up (along the positive Z axis: toward the top of the hand)
        2. E: Move the arm down (along the negative Z axis: toward the palm)

1. **Arm orientation**

    Use SHIFT + WASDQE keys to change the orientation of the arm. This will affect the plane along which the hand will move when changing its position.

        1. SHIFT + W: pitch the arm down 
        2. SHIFT + A: roll the thumb down 
        3. SHIFT + S: pitch the arm up
        4. SHIFT + D: roll the thumb up
        5. SHIFT + Q: yaw toward the thumb
        6. SHIFT + E: yaw away from the thumb

3. **Wrist control**

    Use ZXC keys to control the wrist joint.

        1. Z: Positive rotation
        1. SHIFT + Z: Negative rotation
        1. X: Positive deviation
        1. SHIFT + X: Negative deviation
        1. C: Positive flexion
        1. SHIFT + C: Negative flexion

## Spacenav Control

By default, the spacenav mouse controls the position and
orientation of the viewpoint. Press the button the left side of the mouse
to change control to the arm. Once in this mode, the mouse will change the
position and orientation of the arm. 

## Razer Hydra

Please follow the [hydra installation instructions](http://gazebosim.org/tutorials?tut=hydra&cat=user_input).

Once installed, the right paddle will perform the following actions:

    1. Middle button (between the 1 & 2 buttons): Enable/disable hydra control.
    1. Move paddle to change arm position and orientation.
    1. Button 1: Change to spherical grasp.
    1. Button 2: Change to cylinderical grasp.
    1. Button 3: Change to British pinch grasp.
    1. Button 4: Change to American pinch grasp.
    1. Trigger: Close/open grasp
    1. Bumper: Switch tasks

