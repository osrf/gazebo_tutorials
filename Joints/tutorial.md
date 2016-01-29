# What is a Joint?
A joint connects two links with kinematic and dynamic properties. 
A parent and child relationship is established along with other 
parameters such as axis of rotation, and joint limits. 
Various parameters used in describing any joint can be looked upon, from [here](http://sdformat.org/spec?ver=1.5&elem=joint#joint_parent).

# The various types of joints are: 

1. Revolute : A hinge joint that rotates on a single axis with either a fixed or continuous range of motion.
The range can be set, so that the rotation takes place in a fixed region. 
It is a one-degree-of-freedom joint since it allows rotation only about a single axis.
They are used in door hinges, folding mechanisms, and other uni-axial rotation devices.
They do not allow translation, or sliding linear motion.

[[file:files/revolute.gif|320px]]

The joint joining the mast and the arm, at the point, about which the arm is rotating is a revolute joint.
Observe there is only one plane in which the motion is possible, hence 1 degree of freedom.

2. Revolute2 : Two revolute joints in series are said to be Revolute2 joints.
Simple revolute joints allows motion in plane, via a hinged point. 
Since Revolute2 has two revolute joints, it allows motion in two planes. 

[[file:files/revolute2.gif|320px]]

The wheels of the utility cart above, have Revolute2 joints. One dof enables the
forward and backward motion of the wheels, the other dof enables the wheels to change directions 
i.e instead of moving straight 90o, can take turns and move towards left or right.
The two degrees of freedom are independent of each other.

3. Screw : A single degree of freedom joint with coupled sliding and rotational motion.
Most common usage is to convert rotational motion into linear translation.

[[file:files/screw.gif|320px]]

This is a PR2 gripper. Notice that by the rotatory motion of the motor, the fingertips are translating away from each other.

4. Universal : A universal joint is like a ball and socket joint that constrains an extra degree of rotational freedom.
 Given axis 1 on body 1, and axis 2 on body 2 that is perpendicular to axis 1, it keeps them perpendicular.
 In other words, rotation of the two bodies about the direction perpendicular to the two axes will be equal. 

[[file:files/universal.gif|320px]]

This is a image of cart_front_steer model which uses Universal joint for front left and right wheels.
Observe carefully, there are two directions perpendicular to which motion is possible.
First direction is the direction of rotation of wheels and the second is direction
 perpendicular to the plane of wheels, the latter is used to change the [steering angle](http://street.umn.edu/VehControl/javahelp/HTML/Definition_of_Vehicle_Heading_and_Steeing_Angle.htm) of the cart.

5. Prismatic joint: A sliding joint that slides along an axis with a limited range specified by upper and lower limits. 
The position of one body relative to the other is determined by the distance between two points on a line parallel
to the direction of sliding,with one point fixed in each body. Thus, this joint also has one degree of freedom
Prismatic joints provide single-axis sliding often found in [hydraulic](https://en.wikipedia.org/wiki/Hydraulic_cylinder) and [pneumatic cylinders](https://en.wikipedia.org/wiki/Pneumatic_cylinder).

[[file:files/prismatic.png|320px]]

This is an utility cart, the two cubes shown are brake pedal and gas pedal,
they are connected with the cart using prismatic joint, on applying force on any pedal, 
the pedal slides on the cart and consequently the cart accelerates or decelerates.

[[file:files/prismatic.gif|640px]]
Another example of prismatic is in simple arm.
On applying force on the lower ‘arm_wrist_roll’ of the model, it slides inside the ‘arm_wrist_lift’.

6. Ball joint:

[[file:files/ball.gif|320px]]


7. Fixed:
This can be said as the most simplest of all joints.
It joins two surfaces rigidly with each other, they both appear to be fixed onto each other.

[[file:files/fixed.gif|320px]]

Both of these are model ‘ragdoll’, the right one is pinned to the ground, using fixed joint.
Same force is applied to both of them, but notice,the left one falls on the ground eventually. 
Fixed joint thus, helps to keep things fixed onto each other, another example could be a sensor
attached to a car, the sensor needs to be fixed on to the car.
