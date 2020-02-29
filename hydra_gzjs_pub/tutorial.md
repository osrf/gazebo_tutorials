# Introduction

Hydra publisher using GazeboJs and node-sixense modules. It publishes the hydra messages over the `~/hydra` gazebo topic. Similar tutorial cand be found [here](http://www.gazebosim.org/tutorials?tut=hydra&cat=user_input). The main difference being that the later requires gazebo being installed from source.

## Project setup

Make sure gazebo is running

    gazebo

Install the publisher:

    npm install hydra_gzjs_pub

Run the publisher:

    cd node_modules/hydra_gzjs_pub
    sudo node hydra_pub.js
    
For publishing the hydra controller without root privileges please look at [this](http://www.gazebosim.org/tutorials?tut=hydra&cat=user_input) tutorial.

Test that the publisher works:

    gz topic -l
    gz topic -e /gazebo/default/hydra


### Code
The publisher code is located in the hydra_pub.js file

([hydra_pub.js](https://github.com/ahaidu/hydra_gzjs_pub/raw/master/hydra_pub.js)):

<include src='https://github.com/ahaidu/hydra_gzjs_pub/raw/master/hydra_pub.js' />


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
    var msgS =
~~~

...

~~~
    var msg = JSON.parse(msgS);
    gazebo.publish(type, topic , msg);
});
~~~

### Troubleshooting

In case of offsets, the axis of the controller have been switched in multiple places in order to be similar to the output of the hydra [plugin](http://www.gazebosim.org/tutorials?tut=hydra&cat=user_input) version. If needed otherwise these can be easily changed from the source code.




