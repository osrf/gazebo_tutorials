
## republish

~~~
var util = require('util');
var gazebojs = require('../../gazebojs');
var gazebo = new gazebojs.Gazebo();

if (process.argv.length != 5)
{
  console.log('node camera echo.js [source camera name] [dest camera name] [frame_skip]');
  console.log('ex:\n   node camera chimera 30');
  process.exit(-1);
}


var src_camera = process.argv[2];
var dest_camera = process.argv[3];
var frame_skip = parseInt(process.argv[4]);  

var msg_type = 'gazebo.msgs.ImageStamped';
var src_topic  = '~/' + src_camera + '/link/camera/image';
var dest_topic =  '~/' + dest_camera + '/link/camera/image'; 

var frame_counter = 0;

console.log('== Republishing: [' + src_topic + '] on topic: [' + dest_topic + '] every ' + frame_skip + ' frames');

gazebo.subscribe(msg_type, src_topic,
    function (err, img){
        // make sure the simulation is running
//        pubsub.publish('gazebo.msgs.WorldControl', '~/world_control' , '{"pause": false}');
        if(err)
        {
            console.log('error: ' + err);
            return;
        }
        frame_counter += 1;
        if (frame_counter % frame_skip == 0)
        {
            console.log('publishing frame: ' + frame_counter );
            gazebo.publish( msg_type, dest_topic, img);
        }  

    } );

console.log('setup a loop with 5 sec interval tick');
setInterval(function (){
    console.log('tick');
},5000);
~~~

## Save jpeg


~~~
var util = require('util');
var gazebojs = require('gazebojs');


var fs = require('fs');
var Jpeg = require('jpeg').Jpeg; // https://github.com/pkrumins/node-jpeg

var gz_formats = ['UNKNOWN_PIXEL_FORMAT',
                    'L_INT8',
                    'L_INT16',
                    'RGB_INT8',
                    'RGBA_INT8',
                    'BGRA_INT8',
                    'RGB_INT16',
                    'RGB_INT32',
                    'BGR_INT8',
                    'BGR_INT16',
                    'BGR_INT32',
                    'R_FLOAT16',
                    'RGB_FLOAT16',
                    'R_FLOAT32',
                    'RGB_FLOAT32',
                    'BAYER_RGGB8',
                    'BAYER_RGGR8',
                    'BAYER_GBRG8',
                    'BAYER_GRBG8'];


function getImageInfo(image)
{
    var s = ''
               
    s += 'pixel format: ' + gz_formats[image.pixel_format] + ' [' + image.width + ' x ' + image.height + ']';
    s += ' (bytes per row: ' + image.step + ')';
    return s;
}
 

// adds 0 in front a string for padding
function pad(num, size) {
    var s = num+"";
    while (s.length < size) s = "0" + s;
    return s;
}

function saveToFile(image, fname, cb)
{
    console.log(getImageInfo(image));
    var rgb = new Buffer(image.data, 'base64');
    
    var jpeg = new Jpeg(rgb, image.width, image.height);
    jpeg.encode(function (image) {
        fs.writeFile(fname, image.toString('binary'), 'binary', function(err){
            if(err){
                console.log('ERROR saving file: ' + err);
                process.exit(-2);
            }
            cb(null);
        });  
    });
} 



var fs = require('fs');

if (process.argv.length != 5)
{
  console.log( 'node ' + process.argv[2] + ' [source camera name] [dest_path] [count]');
  
  process.exit(-1);
}

var gazebo = new gazebojs.Gazebo();
var src_camera = process.argv[2];
var dest_path = process.argv[3];
var framesToSave = parseInt(process.argv[4]);  

var msg_type = 'gazebo.msgs.ImageStamped';
var src_topic  = '~/' + src_camera + '/link/camera/image';

var savedFrames = 0;

console.log('saving [' + src_topic + '] to  [' + dest_path + '] for ' + framesToSave + ' frames');

gazebo.subscribe(msg_type, src_topic,
    function (err, img){
        // make sure the simulation is running
        console.log('!');

        if(err) {
            console.log('error: ' + err);
            return;
        }

        if (savedFrames < framesToSave) {
	    // make a nice zero padded number (0003 instead of 3)
	    var nb = pad(savedFrames, 4);
            var fname = dest_path + '_' + nb + '.jpeg' ;
            console.log('saving image: ' + savedFrames );
            console.log('keys:  ' + Object.keys(img));
            console.log('time: ' + util.inspect(img.time));
            saveToFile(img.image, fname, function(err){
                if(err) raise(err);
                console.log(fname + ' saved'); 
                savedFrames += 1;
            });
           console.log('done'); 
        } else {
            process.exit(0);
        }
        
    } );

console.log('setup a loop with 5 sec interval tick');
setInterval(function (){
  console.log('tick');
//  pubsub.publish('gazebo.msgs.WorldControl', '~/world_control' , '{"pause": false}');
},5000);



~~~
