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

## Matlab
Download the MATLAB example, `hxs_example.m`, from
[this link](https://bitbucket.org/osrf/haptix-comm/raw/a6440de8c6c2e1ff0181549145545d7dd604fea5/matlab/hxs_example.m).

To run the MATLAB example, 

## C
Download the `hxs_example.cc` source code here.

## The code explained

### Variable declaration (C)
At the beginning of the C example, we declare all of the structs needed in the
upcoming API calls:

<include from="/hxsSimInfo sim_info;/" to="/int i, j;/" src='http://bitbucket.org/osrf/haptix-comm/raw/update_hxs_example/example/hxs_requester.c'/> 

### `hxs_sim_info`

#### Matlab
~~~
info = hxs_sim_info();
~~~

#### C
<include from="/if \(hxs_sim_info\(&sim_info\) != hxOK\)/" to="/}/" src='http://bitbucket.org/osrf/haptix-comm/raw/update_hxs_example/example/hxs_requester.c'/> 

### `hxs_camera_transform` and `hxs_set_camera_transform`

#### Matlab
<include lang='matlab' from="/tx = hxs_camera_transform\(\);"/ to="/hxs_set_camera_transform\(new_tx\);/" src='http://bitbucket.org/osrf/haptix-comm/raw/default/matlab/hxs_example.m'/> 

#### C
<include from="/if \(hxs_camera_transform\(&camera_transform\) != hxOK\)/" to="/}/" src='http://bitbucket.org/osrf/haptix-comm/raw/update_hxs_example/example/hxs_requester.c'/> 

<include from="/new_transform = camera_transform;/" to="/}/" src='http://bitbucket.org/osrf/haptix-comm/raw/update_hxs_example/example/hxs_requester.c'/> 

