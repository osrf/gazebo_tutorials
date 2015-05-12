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

<include lang='matlab' from="/\% Get the user camera pose/" to="/hxs_set_camera_transform\(new_tx\);/" src='http://bitbucket.org/osrf/haptix-comm/raw/default/matlab/hxs_example.m'/> 

### `hxs_model_color` and `hxs_set_model_color`

<include lang='matlab' from="/\% Change the table color./" to="/hxs_model_color\('table'\);/" src='http://bitbucket.org/osrf/haptix-comm/raw/default/matlab/hxs_example.m'/> 


### `hxs_linear_velocity` and `hxs_set_linear_velocity`
~~~
vel = hxs_linear_velocity('wood_cube_5cm');
~~~

<include lang='matlab' from="/hxs_set_linear_velocity/" to="/pause\(1.0\);/" src='http://bitbucket.org/osrf/haptix-comm/raw/default/matlab/hxs_example.m'/> 

### `hxs_angular_velocity` and `hxs_set_angular_velocity`
~~~
vel = hxs_angular_velocity('wood_cube_5cm');
~~~

<include lang='matlab' from="/hxs_set_angular_velocity/" to="/pause\(1.0\);/" src='http://bitbucket.org/osrf/haptix-comm/raw/default/matlab/hxs_example.m'/> 

### `hxs_apply_force`, `hxs_apply_torque`, and `hxs_apply_wrench`
~~~
hxs_apply_force('wood_cube_5cm', 'link', [-1.0; 0; 0], 0.2);
~~~

~~~
hxs_apply_torque('wood_cube_5cm', 'link', [0; 0; 0.1], 0.1)
~~~

<include lang='matlab' from="/\% Apply force and torque/" to="/pause\(1.5\);/" src='http://bitbucket.org/osrf/haptix-comm/raw/default/matlab/hxs_example.m'/> 


### `hxs_model_gravity_mode` and `hxs_set_model_gravity_mode`

<include lang='matlab' from="/\% Check gravity mode on wooden cube/" to="/hxs_set_model_gravity_mode\('wood_cube_5cm', gravity_mode\);/" src='http://bitbucket.org/osrf/haptix-comm/raw/default/matlab/hxs_example.m'/> 

### `hxs_model_transform` and `hxs_set_model_transform`
<include lang='matlab' from="/\% Get the pose of the cube/" to="/hxs_set_model_transform\('wood_cube_5cm', tx\);/" src='http://bitbucket.org/osrf/haptix-comm/raw/default/matlab/hxs_example.m'/> 

#### Setting the pose of the arm

<include lang='matlab' from="/\% Set the position of the arm/" to="/0, 0, 0\);/" src='http://bitbucket.org/osrf/haptix-comm/raw/default/matlab/hxs_example.m'/> 


### `hxs_model_collide_mode` and `hxs_set_model_collide_mode`
<include lang='matlab' from="/\% Check collide mode on the cube/" to="/hxs_set_model_collide_mode\('wood_cube_5cm', collide_mode\);/" src='http://bitbucket.org/osrf/haptix-comm/raw/default/matlab/hxs_example.m'/> 

### `hxs_add_model` and `hxs_remove_model`

<include lang='matlab' from="/\% Define a new model./" to="/hxs_remove_model\('green_cricket_ball'\);/" src='http://bitbucket.org/osrf/haptix-comm/raw/default/matlab/hxs_example.m'/> 

### `hxs_set_model_joint_state`
<include lang='matlab' from="/\% Set the state of a wrist joint/" to="/pause\(1\);/" src='http://bitbucket.org/osrf/haptix-comm/raw/default/matlab/hxs_example.m'/> 

### `hxs_reset`

<include lang='matlab' from="/\% Move the camera/" to="/hxs_reset\(1\);/" src='http://bitbucket.org/osrf/haptix-comm/raw/default/matlab/hxs_example.m'/> 
