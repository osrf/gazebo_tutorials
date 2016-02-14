## What is a Joint?
A joint connects two links with kinematic and dynamic properties. 
A parent and child relationship is established along with other 
parameters such as axis of rotation, and joint limits. 
Various parameters used in describing any joint can be looked upon, from [here](http://sdformat.org/spec?ver=1.5&elem=joint#joint_parent).

## The various types of joints are: 

# 1. Revolute

[[file:files/revolute.gif|320px]]

The joint joining the mast and the arm, at the point, about which the arm is rotating is a revolute joint.
Observe there is only one plane in which the motion is possible, hence it is a 1 degree of freedom joint.
Here complete 360 degree rotation is taking place but it can be constrained to some limits also.
Door hinges are also example of the same. Notice these joints allow only rotation, no form of linear
translation motion can be forced by them.

# 2. Revolute2 
Two revolute joints in series are said to be Revolute2 joints.
Simple revolute joints allows motion in plane, via a hinged point. 
Since Revolute2 has two revolute joints, it allows motion in two planes. 

[[file:files/revolute2.gif|320px]]

The wheels of the utility cart above, have Revolute2 joints. One degree of freedom enables the
forward and backward motion of the wheels, the other dof enables the wheels to change directions 
i.e instead of moving straight 90o, can take turns and move towards left or right.
The two degrees of freedom are independent of each other.

# 3. Screw 
A single degree of freedom joint with coupled sliding and rotational motion.
Most common usage is to convert rotational motion into linear translation.

[[file:files/screw.gif|320px]]

This is a PR2 gripper. Notice that by the rotatory motion of the motor, the fingertips are translating
away from each other. 

# 4. Universal 
A universal joint is like a ball and socket joint that constrains an extra degree of rotational freedom.

[[file:files/universal.gif|320px]]

This is a image of cart_front_steer model which uses Universal joint for front left and right wheels.
Observe carefully, there are two directions perpendicular to which motion is possible.
First direction is the direction of rotation of wheels and the second is direction
 perpendicular to the plane of wheels, the latter is used to change the [steering angle](http://street.umn.edu/VehControl/javahelp/HTML/Definition_of_Vehicle_Heading_and_Steeing_Angle.htm) of the cart.

# 5. Prismatic joint
A sliding joint that slides along an axis with a limited range specified by upper and lower limits.
One body slides with respect to another body. Notice there can be only one axis of sliding. Since motion is
now restricted only along one axis, this type of joints are One degree of freedom joints.
Examples of prismatic joint can be found in [hydraulic](https://en.wikipedia.org/wiki/Hydraulic_cylinder) and [pneumatic cylinders](https://en.wikipedia.org/wiki/Pneumatic_cylinder).

[[file:files/prismatic.gif|320px]]

An example of prismatic is in simple arm.
On applying force on the lower arm_wrist_roll of the model, it slides inside the arm_wrist_lift.

# 6. Ball joint
This is another 3 degree of freedom joint, where any linear motion is restricted.
This allows only rotatinal motion of the hinged part. In technical terms it the Ball and Socket joint.

[[file:files/ball.gif|320px]]

Here, we have modified the joint connecting one leg and hip of a ragdoll to ball joint.
Observe when force is applied it freely rotates about its own axis and also makes
the leg rotate with itself. (Ofcourse not a good application of ball joint!!!)

# 7. Fixed
This can be said as the most simplest of all joints.
It joins two surfaces rigidly with each other, they both appear to be fixed onto each other.

[[file:files/fixed.gif|320px]]

Both of these are model ragdolls, the right one is pinned to the ground, using fixed joint(torso is fixed to the ground).
Same force is applied to both of them, but notice,the left one falls on the ground eventually. 
Fixed joint thus, helps to keep things fixed onto each other, another example could be a sensor
attached to a car, the sensor needs to be fixed on to the car.


There are few terms which are most commonly used in Joint creation : 
1. Parent-Child : Each joint connects two links, the first link i.e the one on which the jint is hinged is the parent link.
The link which is connected by the joint to the parent is the child.
2. Joint Pose : Similar to link pose, joint pose also has 6 components. Position of x,y and z and orientation: yaw, pitch and
roll, with respect to the current frame of reference. 
3. Axis : Each joint is related to axis. For example revolute joint is hinged to an axis about which the child link roatates. 
Prismatic joint enforces a body to slide along an axis. All the parameters related to such joint axis are defined by this property.
In case of revolute2, there are two axes. In such cases another property called axis2 comes into role.
