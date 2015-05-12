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

<include lang='matlab' from="/\%Get the user camera pose/" to="/hxs_set_camera_transform\(new_tx\);/" src='http://bitbucket.org/osrf/haptix-comm/raw/default/matlab/hxs_example.m'/> 

