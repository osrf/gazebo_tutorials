# Overview

Gazebo communicates on TCP/IP sockets, which allows separate programs to interface with Gazebo. [Boost ASIO](http://www.boost.org/doc/libs/1_53_0/doc/html/boost_asio.html) is used by Gazebo to manage the communication layer, and [Google Protobufs](https://code.google.com/p/protobuf/) are used as the message passing and serialization library. Messages are sent on named channels called **topics** via **publishers**. On the other side of a topic are **subscribers**, which receive callbacks when messages arrive. In summary, to send messages one must publish messages using a publisher on a named topic, and to receive messages one must subscribe to a named topic using a subscriber.

The easiest way to communicate with Gazebo over TCP/IP sockets is to link against the Gazebo libraries, and use the provided functions.

The Gazebo transport system is documented [here](gazebosim.org/api/code/dev/group__gazebo__transport.html).

A list of all the topics in use on a running system can be found with the following command (make sure Gazebo is running first):

~~~
gz topic -l
~~~

# Example

This example subscribes to a [WorldStatistics message](http://gazebosim.org/api/msgs/dev/world__stats_8proto.html) and assumes that you can link against Gazebo.

Download listener.cc and CMakeLists.txt from the above link and put them into to a folder called listener at your home directory. Compile the example:

~~~
mkdir ~/listener
cd ~/listener
wget https://bitbucket.org/osrf/gazebo/raw/default/examples/stand_alone/listener/listener.cc
wget https://bitbucket.org/osrf/gazebo/raw/default/examples/stand_alone/listener/CMakeLists.txt
mkdir build
cd build
cmake ..
make
~~~

Run the listener program when an instance of Gazebo is already running.

~~~
cd ~/listener/build
./listener
~~~

The output should be similar to

~~~
sim_time {
  sec: 1104
  nsec: 855000000
}
pause_time {
  sec: 0
  nsec: 0
}
real_time {
  sec: 1108
  nsec: 263362269
}
paused: false
iterations: 1104855
sim_time {
  sec: 1105
  nsec: 55000000
}
pause_time {
  sec: 0
  nsec: 0
}
real_time {
  sec: 1108
  nsec: 464165998
}
paused: false
iterations: 1105055
~~~

## Explanation of the code ###

Load gazebo and run the transport system.

~~~
gazebo::setupClient(_argc, _argv);
~~~

Next create a Node, which provides functions to create publishers and subscribers.

~~~
gazebo::transport::NodePtr node(new gazebo::transport::Node());
node->Init();
~~~

Create a subscriber on the ''world_stats'' topic. Gazebo publishes a stream of stats on this topic.

~~~
gazebo::transport::SubscriberPtr sub = node->Subscribe("~/world_stats", cb);
~~~

You'll need to create a callback function that will print the message to the console, which we called cb in the previous line.

~~~
void cb(ConstWorldStatisticsPtr &_msg)
{
  std::cout << _msg->DebugString();
}
~~~

At this point you'll have to create a wait loop while messages come in, or do some other processing. Here is simple wait loop.

~~~
while (true)
  gazebo::common::Time::MSleep(10);
~~~

Once you're done, finalize the transport system.

~~~
gazebo::shutdown();
~~~
