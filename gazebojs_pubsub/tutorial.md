# Introduction

This page explains how to create publishers and subscribers to Gazebo topics
in javascript using GazeboJs. Publish / Subscribe is the communication pattern
used between the Gazebo server and its clients.


## Project setup

Unlike software packages that are installed once per machine, NodeJs packages
like Gazebojs are installed inside each node project. Create a NodeJS project
for this tutorial, and install a local copy of the gazebojs package with npm:

    mkdir gazeboJsPubSub
    cd gazeboJsPubSub
    npm init

Just press enter to get the default values. This operation generates a
package.json file. Add gazeboJs to your package file:

    npm install gazebojs --save

Now you can create javascript files and execute them by invoking node.


## Publishers and subscribers

Publishers allow clients and servers to initiate communication, using typed
messages.The messages are defined in Gazebo using Protobuf, and they are
accessed in javascript via a JSON representation. Publishers can be created
in the Gazebo server or the Node client, and messages are sent to unique topics
that subscribers can listen to. Messages can be published to existing topics,
or new topics can be created for future subscribers.


### Node session

Because the code tries to connect to the running simulation server, launch
Gazebo in a separate terminal (if it is not already running) and verify that
the simulation is running (and Sim Time is increasing):

    gazebo

In a separate terminal, from the gazeboJsPubSub directory, start a node
interactive session:

    cd gazeboJsPubSub
    node

And type in the following two lines to load the Gazebo C++ module into the
Node V8 script engine, and crate an instance of the Gazebo class.

~~~
var gazebojs = require("gazebojs")
var gazebo = new gazebojs.Gazebo()
~~~

Type in the following to publish a message:

~~~
gazebo.publish('gazebo.msgs.WorldControl', '~/world_control', { pause: true});
~~~

Type the following to subscribe to the `~/world_stats` topic and send the
ouput to `console.log`.

~~~
gazebo.subscribe('gazebo.msgs.WorldStatistics',  '~/world_stats', console.log)
~~~


The arguments to publish are the message type, the topic name, and the message
(in JSON format) .  Once published, the message is going to be received by
each subscriber for this topic.


Subscribers provide a callback function for a specific type of message on a
certain topic. Each time a new message is published by a publisher, the
callback is invoked for every subscriber to this topic.

It is possible to unsubscribe to a topic. When `unsubscribe` is called, all
subscriptions to that topic are removed.


