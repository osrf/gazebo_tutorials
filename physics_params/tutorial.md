This tutorial is about how to choose or tune physics related
parameters, mainly focused on the physics parameters in Open
Dynamics Engine(ODE).

# Overview
This tutorial first explains physics related parameters that are applicable
to all the physics engines such as `real_time_factor` and `max_step_size`.
Then it explains with demonstration about how to use the parameters for `solvers`,
`constraints`, `friction`. The oder of parameters in this tutorial follows
the `sdformat` specification definition [here](http://sdformat.org/spec?ver=1.6).

# Parameters Applicable to All Physics Engines
The parameters listed in this section are defined under the `<physics>` tag
[here](http://sdformat.org/spec?ver=1.6&elem=physics). These are shared among
all physics engines.

1. `type` The type of the dynamics engine. Currently Gazebo supports 4 physics
engines: `ode`, `bullet`, `simbody` and `dart`. The default physics engine is `ode`.

1. `max_step_size` The maximum time step size during the simulation. `ode` supports
only constant step size and this is a constant value. Default value in Gazebo is
`0.001` seconds.

1. `real_time_factor` `max_step_size x real_time_update_rate` sets an upper bound
of `real_time_factor`. If `real_time_factor < 1` the simulation is slower than real
time.

1. `real_time_update_rate` This is the frequency at which the simulation time steps
are advanced. The default value in Gazebo is `1000` Hz. Multiplying with the default
`max_step_size` of `0.001` seconds gives a `real_time_factor` of `1`. If `real_time_update_rate`
is set to `0` the simulation will run as fast as it can. If Gazebo is not able to update
at the desired rate, it will update as fast as it can, based on the computing power.

1. `max_contacts` The maximum number of contacts to be generated between two entities.
This value can be overwritten by the `max_contacts` element nested under a `collision`
element. This is helpful to constrain the maximum number of contacts between two entities,
such as face-to-face collisions, with a potential sacrifice to the accuracy. The
`max_contacts` parameter is especially helpful for trimesh objects. Another benefit of
`max_contacts` parameter is that it allows users to allocate a fixed amount of memory
at the beginning to store contacts. No reallocation would happen since we have
allocated enough memory to store the maximum number of contacts possible.

# Parameters for Open Dynamics Engine(ODE)
The ODE user guide [here](http://ode.org/ode-latest-userguide.html) includes
the explanation to most physics related parameters. We will refer to this user
guide when necessary.

## Solver parameters
`solver` is an element nested under `physics->ode->solver` in the sdformat
[here](http://sdformat.org/spec?ver=1.6&elem=physics#ode_solver)

1. `type` The type of the solvers in ODE. There are two types: `world` and `quick`.
`world` step uses a direct method called Dantzig, and the `quick` step uses an
iterative Projected Gauss-Seidel (PGS) method. `world` step gives an accurate solution
if it is able to solve the problem, while `quick` step depends on the number of
iterations to reach an accurate enough solution.

    The following two videos show the double pendulum simulation with `world` step and `quick`
    step. It is clear that the contacts between the ground and the double pendulum base are
    stable for `world` step, while unstable (with the contacts disappearing and appearing for
    each time step) for `quick` step.

    <iframe width="560" height="315" src="https://www.youtube.com/embed/vnVbyexorNQ" frameborder="0" gesture="media" allowfullscreen></iframe>
    <iframe width="25" height="315" frameborder="0"></iframe>
    <iframe width="560" height="315" src="https://www.youtube.com/embed/1ncEVDIP1Yo" frameborder="0" gesture="media" allowfullscreen></iframe>

1. `min_step_size` The time duration which advances with each time step of the dynamics
engine. The value of `min_step_size` is no larger than the `max_step_size` under the
`physics` element. If this is left unspecified the default value will be the `max_step_size`
under `physics` element.

1. `iters` The number of iterations for the solver to run for each time step. In general, a
larger number results in better accuracy at the cost of slower simulation, but this is not
guaranteed. Note that this parameter is only meaningful to `quick` solver; `world` solver
doesn't use this parameter.

1. `precon_iters` This parameter only applies to the `quick` solver. It is the number of
iterations used for preconditioning before solving the problem. The preconditioning strategy
is disabled by default and therefore this parameter is not used. As mentioned in the sdformat
specifications this is an experimental parameter.

1. `sor` Successive Over-Relaxation parameter. This is used by `quick` solver only.

1. `use_dynamic_moi_rescaling` Flag to enable dynamic rescaling of moment of inertia in constrained
directions. The implementation of this feature is
[here](https://bitbucket.org/osrf/gazebo/pull-requests/1114)


## Constraints parameters
`constraints` is an element nested under `physics->ode->constraints` in the sdformat
[here](http://sdformat.org/spec?ver=1.6&elem=physics#ode_constraints)

1. `cfm` Constraint Force Mixing (CFM) is a square diagonal matrix. It adds a
small positive value to the matrix in the linear complementarity problem to be solved.
An explanation about `cfm` is [here](http://ode.org/ode-latest-userguide.html#sec_3_8_0).
This should be the first parameter to tune when the simulation is not stable, especially
one with intermittent contacts. CFM stablizes the simulation by converting a hard contact
to a soft one. Therefore, if the error metric to evaluate the solution still uses the violation
of the original hard contact CFM could limit the accuracy of the simulation. Figure 3-5 in
[this paper](https://foswiki.cs.rpi.edu/foswiki/pub/RoboticsWeb/LabPublications/Lu14_IROS.pdf)
gives an explicit demonstration about this limitation. Due to the fact that CFM is always a
small positive value this limitation is always ignored.

1. `erp` Error Reduction Parameter (ERP) specifies what proportion of the joint error will be fixed
during the next simulation step. The local ERP value nested under `joints` overwrites that defined
global. A detailed explanation of `erp` is [here](http://ode.org/ode-latest-userguide.html#sec_3_7_0).
There is also proof about how the `erp`, `cfm` and `kp`, `kd` parameters are equivalent, i.e. `cfm`
and `erp` simulates exactly the spring-damper behavior.

1. `contact_max_correcting_vel`
This is the same parameter as the `max_vel` under `collision->surface->contact`.
`contact_max_correcting_vel` sets `max_vel` globally. Given the following `.world` file example:

    ```
    <?xml version="1.0" ?>
    <sdf version="1.6">
      <world name="default">
        <physics type="ode">
          <ode>
            <solver>
              <type>world</type>
            </solver>
            <constraints>
              <contact_max_correcting_vel>0.1</contact_max_correcting_vel>
              <contact_surface_layer>0.0001</contact_surface_layer>
            </constraints>
          </ode>
          <max_step_size>0.001</max_step_size>
        </physics>
        <gravity>0.0 0.0 -9.81</gravity>
        <include>
          <uri>model://ground_plane</uri>
        </include>
        <include>
          <uri>model://sun</uri>
        </include>
        <model name="sphere_1">
          <pose>0.0 1.8 0.5 0.0 0.0 0.0</pose>
          <link name="link_1">
            <visual name="visual_sphere_1">
              <geometry>
                <sphere>
                 <radius>0.5</radius>
               </sphere>
              </geometry>
              <material>
                <script>Gazebo/WoodPallet</script>
              </material>
            </visual>
            <collision name="collision_sphere_1">
              <geometry>
                <sphere>
                 <radius>0.5</radius>
               </sphere>
              </geometry>
              <surface>
                <contact>
                  <ode>
                    <max_vel>10</max_vel>
                    <min_depth>0.001</min_depth>
                  </ode>
                </contact>
              </surface>
            </collision>
          </link>
        </model>
        <model name="sphere_2">
          <pose>0.0 0.0 0.5 0.0 0.0 0.0</pose>
          <link name="link_2">
            <visual name="visual_sphere_2">
              <geometry>
                <sphere>
                 <radius>0.5</radius>
               </sphere>
              </geometry>
              <material>
                <script>Gazebo/WoodPallet</script>
              </material>
            </visual>
            <collision name="collision_sphere_2">
              <geometry>
                <sphere>
                 <radius>0.5</radius>
               </sphere>
              </geometry>
              <surface>
                <contact>
                  <ode>
                    <max_vel>1</max_vel>
                    <min_depth>0.01</min_depth>
                  </ode>
                </contact>
              </surface>
            </collision>
          </link>
        </model>
      </world>
    </sdf>

    ```
    In the above example, a global `contact_max_correcting_vel` is defined with a value of `0.1`.
    Model `sphere_1` has a `max_vel` of `10` and `sphere_2` has a `max_vel` of `1`. Whenever there
    is a contact between `sphere_1` and `sphere_2`, ODE chose the minimum value of `max_vel` of
    the two entities that form the contact. In this case, `max_vel` for the contact is
    `min(10, 1) = 1`; Finally this `max_vel` is truncated by the `contact_max_correcting_vel`
    defined globally, i.e. the `max_vel` equals to `min(1, 0.1) = 0.1`.

    <iframe width="560" height="315" src="https://www.youtube.com/embed/U12uajzfJUY" frameborder="0" gesture="media" allowfullscreen></iframe>

    This simulation sets `max_vel` to three different values for two spheres in collision. The
    simulation starts with a contact at penetration depth of `0.5`. If we set `max_vel=0` the
    two spheres won't bounce and the velocity is zero after the penetration is corrected. If
    `max_vel` is set to `1` then the red sphere on the top would bounce at a speed no greater
    than `1`, similarly for `max_vel=10`. This is helpful to prevent the simulation from blowing
    up, especially after the simulation corrects some penetration error of contacts.

1. `contact_surface_layer`
This is the same parameter as the `min_depth` under `collision->surface->contact`.
We use the same example as demonstrating `contact_max_correcting_vel` since
`contact_surface_layer` has similar truncation strategy. The penetration depth
between `sphere_1` and `sphere_2` is calculated as `depth` at each time step. The
`min_depth` value is decided by the minimum value between the two entities that
form the contact, and further truncated by the global `contact_surface_layer` value.
In the above example: `min_depth = min(min(0.001, 0.01), 0.0001) = 0.0001`.
With this `min_depth` determined, penetration between the two entities is updated
`depth = max(depth - min_depth, 0)` at each time step.

    <iframe width="560" height="315" src="https://www.youtube.com/embed/LaSlY0cX7qU" frameborder="0" gesture="media" allowfullscreen></iframe>

    Three cases are shown in this video: the leftmost two spheres started with detaching mode,
    i.e. they are not in contact at the beginning of the simulation. The red sphere on the top
    starts falling due to gravity and forms a contact. In this case no penetration would happen
    and therefore `min_depth=0.5` doesn't play any role here. In the second case the two spheres
    start with a contact and a penetration depth of `0.7`, `min_depth=0` in this case. Therefore,
    the simulation corrects the penetration depth of `0.7`. As for the third case, the two spheres
    start with a contact and a penetration depth of `0.7`, `min_depth=0.5`. Therefore, the
    penetration depth will be updated by `depth = max(depth-min_depth, 0)=0.2` and the simulation
    corrects the new `depth` of `0.2`. The two spheres stay with a remaining penetration depth
    equal to the `min_depth`.

## Friction parameters
As for the torsional friction, please refer to the tutorial
[here](http://gazebosim.org/tutorials?tut=torsional_friction&cat=physics).

`friction` is an element nested under `collision->surface->friction` in the sdformat
[here](http://sdformat.org/spec?ver=1.6&elem=collision#friction_ode).

1. `mu` Coefficient of friction along the first friction direction.

1. `mu2` Coefficient of friction along the second friction direction.
For most cases, `mu2` has same value with `mu`.

1. `fdir1` 3-tuple unit vector specifying the direction of `mu` in the
collision local reference frame.

    [[file:files/cone_pyramid.png|460px]]

    This picture shows how the friction cone is approximated with a pyramid model.
    The direction of `fdir1`, `mu` and `mu2` are marked. Whenever a contact forms,
    the normal direction is decided and with a definition of `fdir1`, contact coordinate
    frame can be easily constructed with the third `fdir2` as cross product of unit vector
    along normal and `fdir1` direction.

1. `slip1` Force dependent slip direction 1 in collision local frame.

1. `slip2` Force dependent slip direction 2 in collision local frame.


## Contact parameters
`contact` is an element nested under `collision->surface->contact` in sdformat
[here](http://sdformat.org/spec?ver=1.6&elem=collision#contact_ode).

1. `soft_cfm` Soft constraint force mixing. This is useful to make surfaces soft.

1. `soft_erp` Soft error reduction parameter. This is useful to make surfaces soft.

1. `kp` Dynamically 'stiffness'-equivalent coefficient for contact joints

1. `kd` Dynamically 'damping'-equivalent coefficient for contact joints

    `kp` and `kd` can be used to stabilize the contacts between two entities. The double
    pendulum with `quick` step has unstable intermittent contacts as we compared `quick`
    step with `world` step. If we add `kp` and `kd` parameter to the `collision->surface->contact`
    between the ground and base the contacts will be stable as shown in this following video.

    ```
    <collision name="col_plate_on_ground">
        <pose>0 0 0.01 0 0 0</pose>
        <geometry>
          <cylinder>
            <radius>0.8</radius>
            <length>0.02</length>
          </cylinder>
        </geometry>
        <surface>
          <contact>
              <ode>
                <kp>1e15</kp>
                <kd>1e13</kd>
              </ode>
            </contact>
        </surface>
      </collision>
      ```
    <iframe width="560" height="315" src="https://www.youtube.com/embed/mTN0O5EOOfA" frameborder="0" gesture="media" allowfullscreen></iframe>

1. `max_vel` Maximum correction velocity for contacts. If the predicted velocity for the
next simulation step is larger than this value it will be truncated to this value.
The combination usage with `contact_max_correcting_vel` is demonstrated earlier

1. `min_depth` If the penetration depth is no greater than the value of `min_depth`, then no
contact force would be applied. Otherwise, contact force will be applied to
correct the portion of the penetration with a value of penetration depth minus
`min_depth`. The remaining `min_depth` penetration would stay.


All the world files to generate the demonstration videos are accessible [here](https://bitbucket.org/rosebudflyaway/physics_params_world/overview).
