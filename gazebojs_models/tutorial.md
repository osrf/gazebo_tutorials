
# Introduction

This tutorial shows you how to fetch Gazebo information that is not available via the publishers and subscribers, like the model sdf information.

## Getting SDF information for a model

### Code

Create a NodeJs project

    mkdir gzmodels
    cd gzmodels
    npm install gazebojs

Add a script script file in your project directory:

    gedit model.js

And put the following code inside ([model.js](https://bitbucket.org/osrf/gazebojs/raw/default/examples/model.js))
    
<include src='https://bitbucket.org/osrf/gazebojs/raw/default/examples/model.js' />

### Code explained

First we load the Gazebo bindings into the script engine, and create a client for the running simulation.

~~~
var gazebojs = require('gazebojs')
var sim = new gazebojs.Gazebo();
~~~

Then we use the model function to get the information. The model function takes a model uri (ex: 'model://bowl') and a callback function.
The callback has 2 parameters, error and data. If error is null, then the call was successful and data contains a string with the SDF infromation.

In this case, we simply print the SDF to the console:

~~~
sim.model( model_uri, function (err, data) {
        if(err){
            console.log('Error: ' + err);
        }
        else {
            console.log('data: ' + typeof(data) );
            console.log(data);
       }
    }); 
~~~

### Test your code:

Start Gazebo in a separate terminal. Then you can invoke the script with a model uri as a parameter:

    node model.js model://bowl

You should see the following output:

~~~
information for model: model://bowl

data: string
<?xml version="1.0" ?>
<sdf version="1.5">
  <model name="bowl">
    <link name="link">
      <inertial>
        <mass>0.1</mass>
      </inertial>
      <collision name="collision">
        <geometry>
          <cylinder>
            <radius>0.098</radius>
            <length>0.035</length>
          </cylinder>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <mesh>
            <uri>model://bowl/meshes/bowl.dae</uri>
          </mesh>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>

~~~


## Materials

You can access the materials using Gazebojs. This section shows you how to do it in a node console session. Start the interactive console:

~~~
node
~~~

You can use the Gazebo object to query materials and also the local path of models, using the internal sim object:

~~~
> g = new (require('gazebojs')).Gazebo()

> g.sim.materials()
[ '{"Beer/Diffuse":{"texture":"beer.png"}}',
  '{"Beer/Diffuse":{"texture":"beer.png"}}' ]

> g.sim.modelFile('model://pr2')
'/home/osrf/.gazebo/models/pr2/model.sdf'

~~~

