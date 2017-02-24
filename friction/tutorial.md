# Introduction

Each physics engine in Gazebo (for example ODE, Bullet, Simbody, DART) has
different friction models. Please refer to the [SDF
parameters](http://sdformat.org/spec?ver=1.5&elem=collision#surface_friction) for a complete
listing of available friction parameters.

The rest of this tutorial will assume you are using ODE, the default physics
engine.

# How friction works

When two object collide, such as a ball rolling on a plane, a friction term is generated. In ODE this is composed of two parts, '''mu''' and '''mu2''', where:

  1. '''mu''' is the  Coulomb friction coefficient for the first friction direction, and

  1. '''mu2''' is the friction coefficient for the second friction direction (perpendicular to the first friction direction).

ODE will automatically compute the first and second friction directions for us. Note, you can manually specify the [first friction direction in SDF](http://sdformat.org/spec?ver=1.5&elem=collision#ode_fdir1), but this capability is out of the scope of this tutorial.

The two objects in collision each specify '''mu''' and '''mu2'''. Gazebo will choose the smallest '''mu''' and '''mu2''' from the two colliding objects.

The valid range of values for '''mu''' and '''mu2''' is any non-negative number,
where 0 equates to a friction-less contact
and a large value approximates a surface with infinite friction.
Tables of friction coefficient values for a variety of
materials can be found in engineering handbooks
or [online references](http://www.engineeringtoolbox.com/friction-coefficients-d_778.html).

## How to specify friction

It's always best to refer to the [SDF documentation](http://sdformat.org/spec?ver=1.5&elem=collision#surface_friction).

The following example will specify a box with low friction:

~~~
<model name="box">
  <pose>0 0 0.5 0 0 0</pose>
  <link name="link">
    <collision name="collision">
      <geometry>
        <box>
          <size>1 1 1</size>
        </box>
      </geometry>
      <surface>
        <friction>
          <ode>
            <mu>0.01</mu>
            <mu2>0.01</mu2>
          </ode>
        </friction>
      </surface>
    </collision>
    <visual name="visual">
      <geometry>
        <box>
          <size>1 1 1</size>
        </box>
      </geometry>
    </visual>
  </link>
</model>
~~~

This is a more exhaustive [example](https://bitbucket.org/osrf/gazebo/src/3bd08807f5d9997e9d51eed9276350bac523c4bf/sdf/worlds/test_friction.world?at=default).
