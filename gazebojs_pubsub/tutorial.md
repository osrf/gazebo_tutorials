# Overview

This page explains how to create publishers and subscriber to Gazebo topics in javascript using GazeboJs.


### Project setup

Create a NodeJS project in a new directory:

    mkdir gazeboJsPubSub
    cd gazeboJsPubSub
    npm init

Just press enter to get the default value. This operation generates a package.json file. Add gazeboJs to your package file:

    npm install gazebojs --save

Now you can create javascript files and execute them by invoking node.



### Publishers


Create  publish.js file

    gedit publish.js

And put the following code:

~~~
var gazebojs = require("gazebojs");
var gazebo = new gazebojs.Gazebo();

if (process.argv.length != 5)
{
      console.log('node publish.js [msg type] [topic name] [message]');
        console.log('ex:\nnode publish.js "gazebo.msgs.WorldControl"  "~/world_control" "{\\\"pause\\\": true}"\n');

          process.exit(-1);
}


var type  = process.argv[2];
var topic = process.argv[3];
var msgS   = process.argv[4];

console.log("type:  [" + type + "]");
console.log("topic: [" + topic+ "]");

var msg = JSON.parse(msgS);
console.log("msg:   [" + require('util').inspect(msg)+ "]" ) ;

gazebo.publish(type, topic , msg);
console.log('\npublished!');

setInterval(function (){
      console.log("bye");
        process.exit(0);
},3000);

~~~

Test your installation:

Launch Gazebo in a separate terminal and verify that the simulation is running (Sim Time increases):

    gazebo


Use the 'node' command to invoke the NodeJs REPL console

    node

Type in the following command to load the gazeboJs module, create a simulation client and pause the running simulation

    var gazebojs = require('gazebojs')
    var sim = new gazbojs.Gazebo()
    sim.pause()

You should see the simulation stop in Gazebo.

