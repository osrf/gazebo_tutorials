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

Test your subscriber:


Subscribe for 5 consecutive WorldStatistics messages on the world_stats topic:

    node subscribe.js "gazebo.msgs.WorldStatistics" "~/world_stats" 5

You should see the following output:

~~~
node subscribe.js "gazebo.msgs.WorldStatistics" "~/world_stats" 5
new Gazebo
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


