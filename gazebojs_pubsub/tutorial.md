# Overview

This page explains how to create publishers and subscriber to Gazebo topics in javascript using GazeboJs.


### Project setup

Launch Gazebo in a separate terminal and verify that the simulation is running (Sim Time increases):

    gazebo


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

Test your publisher:


Publish WorldControl message on the world_control topic to pause the simulation:

    node publish.js "gazebo.msgs.WorldControl"  "~/world_control" "{\"pause\": true}"

You should see the simulation stop in Gazebo, and the following output:

~~~
type:  [gazebo.msgs.WorldControl]
topic: [~/world_control]
msg:   [{ pause: true }]

published!
bye
~~~


### Subscribers


Create  subscribe.js file

    gedit subscribe.js

And put the following code:

~~~

var gazebojs = require('../../gazebojs');

if (process.argv.length != 5)
{
    console.log('node subscribe.js [msg type] [topic name] [number of messages]');
    console.log('ex:\nnode subscribe.js "gazebo.msgs.WorldStatistics" "~/world_stats" 10\n');
    process.exit(-1);
}

var type  = process.argv[2];
var topic = process.argv[3];
var count = parseInt(process.argv[4]);

var gazebo= new gazebojs.Gazebo();
console.log("subscribing to topic [" + topic + "] of type [" + type + "]");


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

console.log('keep the process alive...');
setInterval(function (){
    if(count <= 0){
        gazebo.unsubscribe(topic);
        process.exit(0);
   }
},100);

~~~


