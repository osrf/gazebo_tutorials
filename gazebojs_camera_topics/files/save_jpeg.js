
var util = require('util');
var gazebojs = require('gazebojs');
var fs = require('fs');


// adds 0 in front a string for padding
function pad(num, size) {
    var s = num+"";
    while (s.length < size) s = "0" + s;
    return s;
}

if (process.argv.length != 5)
{
  console.log( 'node ' + process.argv[2] + ' [source camera name] [dest_path] [count]');
  process.exit(-1);
}

var gazebo = new gazebojs.Gazebo();
var src_camera = process.argv[2];
var dest_path = process.argv[3];
var framesToSave = parseInt(process.argv[4]);

var src_topic  = '~/' + src_camera + '/link/camera/image';

var savedFrames = 0;

console.log('saving [' + src_topic + '] to  [' + dest_path + '] for ' + framesToSave + ' frames');

options = {format:'jpeg', encoding:'binary' }

gazebo.subscribeToImageTopic(src_topic, function (err, img){
        savedFrames += 1;
        if(err) {
            console.log('error: ' + err);
            return;
        }

        if (savedFrames > framesToSave) {
            console.log('bye');
            process.exit(0);
        }

        // make a nice zero padded number (0003 instead of 3)
        var nb = pad(savedFrames, 4);
        var fname = dest_path + '_' + nb + '.jpeg' ;
        fs.writeFile(fname, img, {encoding:'binary'}, function (err) {
            if(err)
                console.log('ERROR: ' + err);
            else
                console.log(fname + ' saved');
         });

    }, options);

console.log('setup a loop with 5 sec interval tick');
setInterval(function (){
  console.log('tick');
},5000);


