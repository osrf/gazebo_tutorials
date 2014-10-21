# Introduction

This page explains how to create publishers and subscribers to Gazebo topics in javascript using GazeboJs. Publish / Subscribe is the communication pattern used between the Gazebo server and its clients.


## Project setup

Also, because the code tries to connect to the running simulation server, launch Gazebo in a separate terminal (if it is not already running) and verify that the simulation is running (and Sim Time is increasing):

    gazebo

Unlike software packages that are installed once per machine, NodeJs packages like Gazebojs are installed inside each node project. Create a NodeJS project for this tutorial, and install a local copy of the gazebojs package with npm:

    mkdir gazeboJsPubSub
    cd gazeboJsPubSub
    npm init

Just press enter to get the default values. This operation generates a package.json file. Add gazeboJs to your package file:

    npm install gazebojs --save

Now you can create javascript files and execute them by invoking node.



## Publishers

Publishers allow clients and servers to initiate communication, using typed messages. The messages are defined in Gazebo using Protobuf, and they are accessed in javascript via a JSON representation. Publishers can be created in the Gazebo server or the Node client, and messages are sent to unique topics that subscribers can listen to. Messages can be published to existing topics, or new topics can be created for future subscribers.


### Code
Create  publish.js file

    gedit publish.js

And put the following code:

<include src='https://bitbucket.org/osrf/gazebojs/raw/default/examples/publish.js' />


### Code explained

The first two lines load the Gazebo C++ module into the Node V8 script engine, and an instance of the Gazebo class is created.

~~~
var gazebojs = require("gazebojs");
var gazebo = new gazebojs.Gazebo();
~~~

The command line arguments are then parsed to determine the type of protobuf Gazebo message to send, an acual instance of a message encoded in JSON, and the topic on which to send the message.
This information is then used to publish a message:

~~~
gazebo.publish(type, topic , msg);
~~~

Once published, the message is going to be received by each subscriber for this topic.
 

### Test your publisher:


Publish `WorldControl` message on the world_control topic to pause the simulation:

    node publish.js "gazebo.msgs.WorldControl"  "~/world_control" "{\"pause\": true}"

You should see the simulation stop in Gazebo, and the following output:

~~~
type:  [gazebo.msgs.WorldControl]
topic: [~/world_control]
msg:   [{ pause: true }]

published!
bye
~~~


## Subscribers

Subscribers provide a callback function for a specific type of message on a certain topic. Each time a new message is published by a publisher, the callback is invoked for every subscriber to this topic.

### Code
Create subscribe.js file

    gedit subscribe.js

And put the following code:


<include src='https://bitbucket.org/osrf/gazebojs/raw/default/examples/subscribe.js' />

### Code explained

A subscriber is created by calling subscribe with a message type (the protobuf message name), the topic and a callback function. The call to subscribe is non blocking. The callback has 2 parameters, error and data... following the NodeJs pattern for asynchronous execution.
In this example, the callback simply converts the JSON to a string and prints it on the console.

~~~
// subscribe to the topic with a callback function
gazebo.subscribe(type, topic, function (err, msg){
    try {
        if (err) throw(err);
        console.log('-- [' + count + '] --');
        count += -1;
        // convert the Json msg to a string
        var s= JSON.stringify(msg);
        console.log(s);
    } catch(err)  {
        console.log('error: ' + err);
        console.log(msg);
   }
});
~~~

It is possible to unsubscribe to a topic. When unsubscribe is called, all subscriptions to that topic are removed. If you need more than one subscriber on the same topic and you don't want to unsubscribe to them at the same time, you need to use multiple instances of gazebojs.Gazebo.

### Test your subscriber:


Subscribe for 5 consecutive `WorldStatistics` messages on the world_stats topic:

    node subscribe.js "gazebo.msgs.WorldStatistics" "~/world_stats" 5

You should see the following output:

~~~
node subscribe.js "gazebo.msgs.WorldStatistics" "~/world_stats" 5

subscribing to topic [~/world_stats] of type [gazebo.msgs.WorldStatistics]
keep the process alive...
-- [5] --
{"sim_time":{"sec":13081,"nsec":864000000},"iterations":13081864,"paused":false,"pause_time":{"sec":69,"nsec":653000000},"real_time":{"sec":13115,"nsec":231224650}}
-- [4] --
{"sim_time":{"sec":13082,"nsec":64000000},"iterations":13082064,"paused":false,"pause_time":{"sec":69,"nsec":653000000},"real_time":{"sec":13115,"nsec":431904634}}
-- [3] --
{"sim_time":{"sec":13082,"nsec":264000000},"iterations":13082264,"paused":false,"pause_time":{"sec":69,"nsec":653000000},"real_time":{"sec":13115,"nsec":632504383}}
-- [2] --
{"sim_time":{"sec":13082,"nsec":464000000},"iterations":13082464,"paused":false,"pause_time":{"sec":69,"nsec":653000000},"real_time":{"sec":13115,"nsec":833144611}}
-- [1] --
{"sim_time":{"sec":13082,"nsec":664000000},"iterations":13082664,"paused":false,"pause_time":{"sec":69,"nsec":653000000},"real_time":{"sec":13116,"nsec":33668964}}
GZPubSub::Unsubscribe() topic = [~/world_stats]
~~~


