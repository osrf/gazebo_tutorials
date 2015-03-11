# Tutorial: Apply Force and Torque via GUI

## Introduction

This tutorial will explain how to apply force and/or torque to models during the simulation using the graphical user interface.

## Applying force and torque examples

Let's go through an example of applying force and torque to simple models. Open Gazebo and from the insert tab, insert a `Simple Arm` into the scene. Then, from the top toolbar, insert a box. Make sure the simulation is not paused.

### Apply force to a link's center of mass

We want to apply force to a specific link in the `Simple Arm` model. On the World tree, right-click `arm_wrist_lift` and choose `Apply Force/Torque`. A dialog will pop up and you'll see a straight arrow and a curved arrow attach to the arm.

On the dialog, write `100 N` on the `Y` field under Force and press `Enter`, the arm will start rotating slightly.

### Apply torque to a link

Under torque, write `100 Nm` on the `Z` field and press `Apply Torque`. The arm will rotate slightly.

### Apply force with an offset

Now let's apply force to the box. Right-click the box and choose `Apply Force/Torque`. A new dialog will pop up.

Under `Force`, type `1000 N` on the `X` field. Then under `Application point`, press the up arrow in the `Y` field until it reaches `1 m`. Press `Enter` and the box will rotate slightly. 

## The dialog explained

* *Apply to link:* Choose which link in the current model to apply force and torque to. When you choose a different link, the arrows will move to that link. Note that you can have many dialogs open and attached to different links or even to the same link, but you can't apply force with more than one dialog at once.

### *Force*

    * *X, Y, Z*: Each field specifies how much force will be applied on that direction. The frame is fixed to the link.

    * *Mag*: The total magnitude of the force which will be applied, which is the Euclidean norm of the 3 forces above. Changing the magnitude changes the XYZ fields proportionally, maintaining the force direction.

    * *Clear*: Pressing this button will zero the `X`, `Y`, `Z` and `Mag` fields.

    * *Application point*: By default, force is applied to the link's center of mass. Here you can edit the `X`, `Y` and `Z` fields to give the force an offset with respect to the link's origin expressed in the link's frame. Select the `Center of mass` again to go back to applying force to it. If the center of mass doesn't coincide with the link origin, the XYZ fields will be updated to match the it.

    > *Tip*: Right-click the model and choose `View -> Center of mass` to see its position. 
  
    * *Apply Force*: Click this to apply just the force for one time step. 

### *Torque*

    * *X, Y, Z*: Each field specifies how much torque will be applied about that axis. The frame is fixed to the link.

    * *Mag*: The total magnitude of the torque which will be applied, which is the Euclidean norm of the 3 torques above. Changing the magnitude changes the XYZ fields proportionally, maintaining the torque direction.

    * *Clear*: Pressing this button will zero the `X`, `Y`, `Z` and `Mag` fields.

    > Note: Torque is always applied about the center of mass.

### *Apply All*

Force and torque are applied at the same time. This button can 

