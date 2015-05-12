# Overview
This tutorial gives an overview of the haptix-comm simulation-specific API.

We assume that you have already completed the
[installation step](http://gazebosim.org/tutorials?tut=haptix_install&cat=haptix).

# Documentation
The full sim API documentation can be found
[here](https://s3.amazonaws.com/osrf-distributions/haptix/api/0.2.2/haptix__sim_8h.html).

((Flesh out high level))

# Example
In our example, we will demonstrate how to use the sim API to manipulate objects in the ARAT world.

Download the MATLAB example, `hxs_example.m`, from
[this link](https://bitbucket.org/osrf/haptix-comm/raw/default/matlab/hxs_example.m).

To run the MATLAB example, 


## The code explained

### `hxs_sim_info`

~~~
info = hxs_sim_info();
~~~

### `hxs_camera_transform` and `hxs_set_camera_transform`

<include lang='matlab' from="/% Get the user camera pose/" to="/hxs_set_camera_transform\(new_tx\);/" src='http://bitbucket.org/osrf/haptix-comm/raw/default/matlab/hxs_example.m'/> 

### `hxs_model_color` and `hxs_set_model_color`

<include lang='matlab' from="/% Change the table color./" to="/hxs_model_color\('table'\);/" src='http://bitbucket.org/osrf/haptix-comm/raw/default/matlab/hxs_example.m'/> 


### `hxs_linear_velocity` and `hxs_set_linear_velocity`

~~~
vel = hxs_linear_velocity('wood_cube_5cm');
~~~

~~~
% Move by setting linear velocity
hxs_set_linear_velocity('wood_cube_5cm', [-0.5; 0; 0]);
~~~

### `hxs_angular_velocity` and `hxs_set_angular_velocity`

~~~
vel = hxs_angular_velocity('wood_cube_5cm');
~~~

~~~
% Move by setting angular velocity
hxs_set_angular_velocity('wood_cube_5cm', [0; 0; 100]);
~~~

### `hxs_apply_force`, `hxs_apply_torque`, and `hxs_apply_wrench`

~~~
hxs_apply_force('wood_cube_5cm', 'link', [-1.0; 0; 0], 0.2);
~~~

~~~
hxs_apply_torque('wood_cube_5cm', 'link', [0; 0; 0.1], 0.1)
~~~


<include lang='matlab' from="/% Apply force and torque at the same time./" to="/pause\(1.5\);/" src='http://bitbucket.org/osrf/haptix-comm/raw/default/matlab/hxs_example.m'/> 


