
## model.js

~~~
var gazebojs = require('../../gazebojs')
var sim = new gazebojs.Gazebo();

var model_uri  = process.argv[2];

if ( process.argv.length != 3 ) {
    console.log('node model.js [model_uri]');
    console.log('ex:\nnode model.js model://bowl');
    process.exit(-1);
}

console.log('information for model: ' + model_uri + '\n');

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

