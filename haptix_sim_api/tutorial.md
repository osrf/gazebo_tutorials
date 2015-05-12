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
[this link](https://bitbucket.org/osrf/haptix-comm/raw/a6440de8c6c2e1ff0181549145545d7dd604fea5/matlab/hxs_example.m).

To run the MATLAB example, 


## The code explained

### `hxs_sim_info`

~~~
info = hxs_sim_info();
~~~

### `hxs_camera_transform` and `hxs_set_camera_transform`

<include lang='matlab' from="/tx = hxs_camera_transform\(\);/" to="/hxs_set_camera_transform\(new_tx\);/" src='http://bitbucket.org/osrf/haptix-comm/raw/default/matlab/hxs_example.m'/> 

### `hxs_model_color` and `hxs_set_model_color`

<include lang='matlab' from="/hxs_set_model_color/" to="/hxs_model_color\('table'\);/" src='http://bitbucket.org/osrf/haptix-comm/raw/default/matlab/hxs_example.m'/> 


### `hxs_linear_velocity` and `hxs_set_linear_velocity`

### `hxs_angular_velocity` and `hxs_set_angular_velocity`

### `hxs_apply_force`, `hxs_apply_torque`, and `hxs_apply_wrench`

### `hxs_model_gravity_mode` and `hxs_set_model_gravity_mode`

### `hxs_model_transform` and `hxs_set_model_transform`

### `hxs_model_collide_mode` and `hxs_set_model_collide_mode`

### `hxs_add_model` and `hxs_remove_model

### `hxs_set_model_joint_state`

### `hxs_reset`

