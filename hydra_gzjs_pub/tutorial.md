# Introduction

Hydra publisher using GazeboJs and node-sixense modules. 


## Project setup

Make sure gazebo is running

    gazebo

Install the publisher:

    npm install hydra_gzjs_pub

Run the publisher:

    cd node_modules/hydra_gzjs_pub
    sudo node hydra_pub.js

Test that the publisher works:

    gz topic -l
    gz topic -e /gazebo/<my_world>/hydra


### Code
The publisher code is located in the hydra_pub.js file

([hydra_pub.js](https://bitbucket.org/ahaidu/hydra_gzjs_pub/raw/master/hydra_pub.js)):

<include src='https://bitbucket.org/ahaidu/hydra_gzjs_pub/raw/master/hydra_pub.js' />


### Code explained

The first lines load the Gazebo C++ and the node-sixense modules into the Node V8 script engine, and creates and instance of the Gazebo class.

~~~
var hydra =  require('node-sixense');  
var gazebojs = require('gazebojs');
var gazebo = new gazebojs.Gazebo();
~~~

We then set the message type and the topic:

~~~
var type = "gazebo.msgs.Hydra"
var topic = "~/hydra"
~~~

We then initialize sixense and for every new data we call the `sixenseGetAllNewestDataAsync` callback function which publishes the hydra message.

~~~
hydra.sixenseInit();
hydra.sixenseSetActiveBase(0);
hydra.sixenseGetAllNewestDataAsync(function (error, allData)
{    
    var msgS = <...>
    var msg = JSON.parse(msgS);
    gazebo.publish(type, topic , msg);
});
~~~



