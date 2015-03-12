# Tutorial: Apply Force and Torque via GUI

## Introduction

This tutorial will explain how to apply force and/or torque to models during the simulation using the graphical user interface.

## Applying force and torque examples

Let's go through an example of applying force and torque to simple models. Open Gazebo and from the insert tab, insert a `Simple Arm` into the scene. Then, from the top toolbar, insert a box. Make sure the simulation is not paused.

[[file:files/insert_models.png|800px]]

### Apply force to a link

We want to apply force to a specific link in the `Simple Arm` model. On the World tree, right-click `arm_wrist_lift` and choose `Apply Force/Torque`. A dialog will pop up and you'll see a straight arrow and a curved arrow attach to the arm.

On the dialog, write `100 N` on the `Y` field under `Force` and press `Enter`, the arm will start rotating slightly.

[[file:files/apply_force.png|800px]]

### Apply torque to a link

On `Apply to link`, select `arm_elbow_pan`, the arrows will move to this link. Under torque, write `100 Nm` on the `Z` field and press `Apply Torque` a few times to see the arm rotate slightly.

[[file:files/apply_torque.png|800px]]

### Apply force with an offset

Now let's apply force to the box. Right-click the box in the scene and choose `Apply Force/Torque`. A new dialog will pop up.

Under `Force`, type `1000 N` on the `X` field. Then under `Application point`, press the up arrow in the `Y` field until it reaches `1 m`, you'll see the arrow moving as you do it. Press `Enter` a few times to and the box will rotate. Hold `Enter` to repeatedly apply the force and make the box spin faster.

[[file:files/apply_force_offset.png|800px]]

## The interface explained

### Force

* **X, Y, Z**: Each field specifies how much force will be applied on that direction. The frame is fixed to the link.

* **Mag**: The total magnitude of the force which will be applied, which is the Euclidean norm of the 3 forces above. Changing the magnitude changes the `XYZ` fields proportionally, maintaining the force direction.

* **Clear**: Pressing this button will zero the `X`, `Y`, `Z` and `Mag` fields.

* **Application point**: By default, force is applied to the link's center of mass. Here you can edit the `X`, `Y` and `Z` fields to give the force an offset with respect to the link's origin expressed in the link's frame. Select `Center of mass` again to fill the `XYZ` fields withits coordinates.

    > **Tip**: Right-click the model and choose `View -> Center of mass` to see its position. You might want to also make the model transparent for that.

* **Apply Force**: Click this to apply only force for one time step. Keep in mind that time steps are in the order of miliseconds, so relatively large forces are needed in order to apply a significant impulse.

### Torque

* **X, Y, Z**: Each field specifies how much torque will be applied about that axis. The frame is fixed to the link.

* **Mag**: The total magnitude of the torque which will be applied, which is the Euclidean norm of the 3 torques above. Changing the magnitude changes the `XYZ` fields proportionally, maintaining the torque direction.

* **Clear**: Pressing this button will zero the `X`, `Y`, `Z` and `Mag` fields.

* **Apply Torque**: Click this to apply only torque for one time step. Keep in mind that time steps are in the order of miliseconds, so relatively large torques are needed in order to apply a significant angular impulse.

    > **Note**: Torque is always applied about the center of mass.

### Apply All

Force and torque are applied at the same time, i.e. apply a wrench. Hold enter to repeatededly trigger this button.

> **Note**: If you apply force and/or torque while the simulation is paused, they will accumulate and be applied all at once when the simulation is unpaused.

### Rotation tool

The arrows directions will always match the directions specified in the dialog. From the dialog, the direction can be changed by editing the numbers on the `XYZ` fields. From the scene, the direction can be changed by dragging the pink circles around the arrows.

> **Tip**: The pink circles are attached to the highlighted arrow. To rotate the other arrow, first click it to attach the circles to it and then drag them normally.

