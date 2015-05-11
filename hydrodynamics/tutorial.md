# Overview

This tutorial describes how to use the `BuoyancyPlugin` in concert with the
`LiftDragPlugin` to simulate the behavior of underwater objects.

## Demo

First, open the `underwater.world` environment with Gazebo paused:

```
gazebo --verbose worlds/underwater.world -u
```

(((picture)))

This world contains three submarines. The white submarine will float to the top
of the world, the black submarine will sink to the ground plane, and the yellow
submarine will maintain a constant height if left undisturbed.

Each submarine model has a propeller link that will move the submarine forward
when the joint between the propeller and the body is spun.

Drag out the left hand menu and select the yellow submarine model. Under the
"velocity" tab, give the `spinning_joint` joint a 

(((picture)))

## Explanation


