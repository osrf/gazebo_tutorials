# Overview

This tutorial will teach you how to make your own "tactor glove" for translating sensor data from the HAPTIX simulator to real tactile feedback.

# What You Need

Hardware
--------
  + 5 [Lilypad Vibe Boards](https://www.sparkfun.com/products/11008)
  + 1 [Teensy USB Board](https://www.pjrc.com/teensy/)
  + 1 Mini-B USB cable (available from Amazon, Radioshack, etc.)
  + Wires (we used 30 gauge teflon)
  + Soldering iron
  + 1 glove, preferably made of a thin spandex material
  + Needle and thread (or a sewing machine)

Software
--------
You will need a Linux machine with Gazebo and the handsim package. See [this tutorial](http://gazebosim.org/tutorials?tut=haptix_install&cat=haptix) for installation instructions.

To program the Teensy USB board, you will need to install the [Arduino IDE](http://arduino.cc/en/Main/Software) and the [Teensyduino add-on](https://www.pjrc.com/teensy/teensyduino.html) for Arduino.

The machine talking to the glove must be running Linux (preferably Ubuntu) and must have the haptix-comm library installed (see [here](https://www.pjrc.com/teensy/teensyduino.html) for installation instructions). It can be the same machine that is running Gazebo, but it doesn't have to be.

# Design the Glove

This might be the most difficult part of this tutorial, and the most fun.

The basic idea behind the tactor glove is to attach the 5 Lilypad boards to each finger of the glove. The placement of the Lilypad on each finger, the method of attachment for the boards, and type of glove (full-fingered vs. fingerless) are all up to you. In fact, instead of a glove you could use an armband (particularly if you are, say, designing a haptic feedback system for a person preparing for a real prosthetic arm). You'll also want to think about where to put the USB board that acts as the hub for the Lilypads and connects to the computer. And if you don't know how to sew, well... there's always hot glue, right?

We started with a full-fingered glove and sewed pockets on the lower third of each finger to put the Lilypads. The pockets were snug enough against the fabric that the Lilypads were in no danger of falling out. The USB board was put into a pocket on the back of the hand, and the USB cable was fed through a small hole near the bottom of the glove.

[[file:files/glove_back.jpg|600px]]

[[file:files/glove_front.jpg|600px]]

The wires (two per each board) were twisted together and joined at the back of the hand to minimize trailing.

# Assembling the Electronics

Measure out two wires for each Lilypad. The length of the wires depends on the finger placement of the motors and the placement of the board. We suggest at least 8 inches (a bit extra doesn't hurt).

For each Lilypad, connect the negative (-) terminal to the GND pin of the Teensy board, and the positive (+) terminal to a unique numbered pin of your choice. We chose pins 4, 9, 10, 12, and 14. Make sure you take note of which pins you use, as it will effect the Arduino code in the next section.

[[file:files/teensy_pinout.png|402px]]

Connect the Mini-USB cable to the port on the Teensy board.

Optional: We covered the Lilypad boards and the USB board in shrink wrap. This serves several purposes: it takes stress off the wires, which are already quite thin, it isolates the components from electrostatic interference originating from the fabric, and it provides some protection in case somebody spills Mountain Dew on your lovingly crafted tactor glove.

[[file:files/tactors_soldered.jpg|600px]]

Now that everything is soldered up and connected, it's time to make the Lilypads buzz. 

# Programming the Board
Open the Arduino IDE and start a new sketch. Click on the 'Tools' drop-down menu and select 'USB Type: "Serial"'.

Download the Arduino sketch code from [here](http://bitbucket.org/osrf/gazebo_tutorials/raw/haptix_tactors/haptix_tactors/files/arduino_sketch.c) and copy it into the Arduino IDE.

This Arduino sketch behaves as follows:

When the USB board gets plugged into a Linux computer, it will create an ACM device (e.g. "/dev/ttyACM0"). It will listen for characters written to that device. When characters '1' through '5' are written to the device, the corresponding motor will buzz for 100 milliseconds.

You may want to customize this code depending on your electronics setup or other preferences.

~~~
#define NUM_MOTORS 5
~~~

If you have more than 5 motors, you should change this number to match.

~~~
#define MOTOR_RUN_TIME 100
~~~

This line defines the length of one motor 'click' in milliseconds.

~~~
int motors[NUM_MOTORS] = {4, 9, 10, 12, 14};
~~~

Make sure these numbers correspond to the pins that you chose to connect the Lilypad boards to in the previous section.

~~~
unsigned long t_motor_start[NUM_MOTORS] = {0, 0, 0, 0, 0};
~~~

If you change the number of motors, change the number of zeros in this array.

~~~
if (b >= '1' && b <= '5')
{
  int motor = b - '1';
  t_motor_start[motor] = millis();
  analogWrite(motors[motor], MOTOR_PWM);
}
~~~

You can modify this section to change which characters make the motors buzz. For example, you could do 'a', 'b', 'c'... instead of 1-5. You should also change the limit in the 'if' statement if you change the number of motors.

Build the project and use the [Teensy bootloader](https://www.pjrc.com/teensy/loader.html) to load the sketch onto your board.

# Testing the Board
You can easily test to see if your electronics rig and your software work using picocom.

Install picocom by bringing up a terminal and running

~~~
sudo apt-get install picocom
~~~

Plug the tactor glove into the USB port of your machine and run

~~~
picocom /dev/ttyACM0
~~~

You should be able to press the number keys 1-5 (or whatever keys corresponding to the characters you chose in the previous step) to make the motors buzz. Press CTRL-a CTRL-q to exit.

If `/dev/ttyACM0` fails to open, then `ls /dev` to see if another ACM device is available.

You might need to run picocom as root (`sudo picocom`). You will need to add your username to the `plugdev` and `dialout` groups to open devices without sudoing.

~~~
sudo usermod -a -G plugdev <username>
sudo usermod -a -G dialout <username>
~~~

Make sure you include `-a` in the command, otherwise your account could lose sudo access!

Once you have confirmed that the electronics and the Arduino sketch are working, execute whatever plans you made for attaching the motors to the glove in the first step. Make sure you know the correspondence between the motor on each finger and which character will make it buzz. For reference, here are the mappings we used:

|Finger |Teensy Board Pin| Character|
|:------|:--------------:|---------:|
|Index  |     4          | 1        |
|Middle |     9          | 2        |
|Ring   |     10         | 3        |
|Little |     12         | 4        |
|Thumb  |     14         | 5        |

# Communicating with Gazebo
We are going to write a haptix-comm client that reads contact sensor data from the simulation and translate it to motor clicks.

Make a folder for your C code and download the tactor glove [source code](http://bitbucket.org/osrf/gazebo_tutorials/raw/haptix_tactors/haptix_tactors/files/tactors.cc) and the [CMakeLists.txt](http://bitbucket.org/osrf/gazebo_tutorials/raw/haptix_tactors/haptix_tactors/files/CMakeLists.txt) for this project.

Again, you may need to customize some parts of the code if you have a different electronics setup from this example.

~~~
const unsigned int sleeptime_us = 10000;
const float minContactForce = 0;
~~~

`sleeptime_us` controls the rate of updates from the simulator. The default value is 10000 microseconds. `minContactForce` is the minimum force a contact sensor must experience to make the corresponding motor click.

~~~
int fd = open("/dev/ttyACM0", O_WRONLY);
if (fd < 0)
{
  perror("Failed to open /dev/ttyACM0");
  return -1;
}
~~~

You may want to change which device the program opens, especially if you have other ACM devices that have already taken `/dev/tty/ACM0`.

~~~
for (unsigned int i = 0; i < deviceInfo.ncontactsensor; i++)
{
  // Uninitialized
  sensorMotorIndexMapping[i] = '0';
}
for (unsigned int i = 3; i <= 6; i++)
{
  // Index
  sensorMotorIndexMapping[i] = '1';
}
for (unsigned int i = 11; i <= 14; i++)
{
  // Middle
  sensorMotorIndexMapping[i] = '2';
}
for (unsigned int i = 17; i <= 20; i++)
{
  // Ring
  sensorMotorIndexMapping[i] = '3';
}
for (unsigned int i = 7; i <= 9; i++)
{
  // Little
  sensorMotorIndexMapping[i] = '4';
}
for (unsigned int i = 21; i <= 23; i++)
{
  // Thumb
  sensorMotorIndexMapping[i] = '5';
}
~~~

This part of the code maps the indices of the contact sensors on each finger of the MPL arm to the character that will make the motor buzz when written to `/dev/ttyACM0`. Change the value of the map entries if you changed the characters in the Arduino sketch.

~~~
if (j <= '5' && j >= '1')
~~~

You may need to change these bounds if you have a different range for characters that map to motors.

After any necessary modifications, run the following commands in the folder with your code:

~~~
mkdir build;
cd build;
cmake ..;
make
~~~

In a separate window, run the Gazebo HAPTIX simulator:

~~~
gazebo worlds/arat.worlds
~~~

Then in the folder with your tactors code, execute the tactors executable. Put on the glove and try picking up items in the simulator.

[[file:files/grasp_sim.png|700px]]
