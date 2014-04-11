#Tutorial: Modifying a World#

This tutorial describes how to modify global properties including scene and physics properties.

Start gazebo:

~~~
gazebo
~~~

##Scene Properties##

In the `World` tab, select the `scene` item. A list of scene properties will be displayed in the list box below. Click the triangle to expand the properties.

<a href="http://gazebosim.org/w/images/3/38/TutorialSceneTab.png" class="image"><img alt="TutorialSceneTab.png" src="http://gazebosim.org/w/images/3/38/TutorialSceneTab.png" width="268" height="687"></a>
[[File:TutorialSceneTab.png]] [[File:TutorialSceneTabExpanded.png]]

These properties allow you to change the ambient light. Note: The background color will not change if the Sky is enabled.



##Physics Properties##


In the `World` tab, select the `physics` item. A list of physics properties will be displayed in the list box below.

* The `enable physics` check-box can be used to disable physics while allowing plugins and sensors to continue running.
* The `real time update rate` parameter specifies in Hz the number of physics updates that will be attempted per second. If this number is set to zero, it will run as fast as it can. Note that the product of `real time update rate` and `max step size` represents the target `real time factor`, or ratio of simulation time to real-time.
* The `max step size` specifies the time duration in seconds of each physics update step.

In the gravity block:
* The `x`, `y` and `z` parameters set the global gravity vector components in m/s^2.

In the solver block:

* The `iterations` parameter specifies the number of iterations to use for iterative LCP solvers (used by ODE and bullet).
* The `SOR` parameter stands for [[wikipedia:Successive_over-relaxation | successive over-relaxation]], which can be used to try to speed the convergence of the iterative method.

The constraints block contains several parameters related to solving constraints:

* The `CFM` and `ERP` parameters stands for [http://ode-wiki.org/wiki/index.php?title=Manual:\_Concepts#Constraint_Force_Mixing_.28CFM.29 Constraint Force Mixing] and [http://ode-wiki.org/wiki/index.php?title=Manual:\_Concepts#Joint_error_and_the_Error_Reduction_Parameter_.28ERP.29 Error Reduction Parameter] and are used by ODE and bullet. The CFM and ERP parameters [http://ode-wiki.org/wiki/index.php?title=Manual:\_Concepts#How_To_Use_ERP_and_CFM can be related to linear stiffness and damping coefficients]. The `max velocity` and `surface layer` parameters are used to resolve contacts with a split impulse method. Any contacts with that penetrate deeper than a depth specified by `surface layer` and have a normal velocity less than `max velocity` will not bounce.

See the [http://gazebosim.org/sdf/dev.html#physics sdf physics documentation] for a description of these parameters.

<a href="http://gazebosim.org/w/images/a/ad/TutorialPhysicsTab.png" class="image"><img alt="TutorialPhysicsTab.png" src="http://gazebosim.org/w/images/a/ad/TutorialPhysicsTab.png" width="330" height="782"></a>
[[File:TutorialPhysicsTab.png]]

## Next

Next: [Next: Manipulate a world through a plugin](http://gazebosim.org/tutorials/?tut=plugin_world)
