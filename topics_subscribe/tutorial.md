#Tutorial: How to subscribe to gazebo topics#

Gazebo communicates on TCP/IP sockets, which allows separate programs to interface with Gazebo. [http://www.boost.org/doc/libs/1_53_0/doc/html/boost_asio.html Boost ASIO] is used by Gazebo to manage the communication layer, and [https://code.google.com/p/protobuf/ Google Protobufs] are used as the message passing and serialization library. Messages are sent on named channels called **topics** via **publishers**. On the other side of a topic are **subscribers**, which receive callbacks when messages arrive. In summary, to send messages one must publish messages using a publisher on a named topic, and to receive messages one must subscribe to a named topic using a subscriber.

The easiest way to communicate with Gazebo over TCP/IP sockets is to link against the Gazebo libraries, and use the provided functions.

The Gazebo transport system is documented [http://gazebosim.org/api/1.9.0/group__gazebo__msgs.html here] and the protobuf message types are documented [http://gazebosim.org/msgs/1.9.0/classes.html here].

A list of all the topics in use on a running system can be found with the following command (make sure Gazebo is running first):

        gztopic list

### Example ###

This example subscribes to a [http://gazebosim.org/msgs/1.9.0/world__stats_8proto.html WorldStatistics message] on topic ~/world_stats and prints it to the console. This message is output by Gazebo at 1 Hz and includes information about the simulation time and pause state. The example is included in the gazebo source distribution at [https://bitbucket.org/osrf/gazebo/src/8b0c2de0886a61a862036f7e7fe1d4b7b8b4651c/examples/stand_alone/listener/listener.cc?at=gazebo_1.9 examples/stand_alone/listener/] and assumes that you can link against Gazebo.

Download listener.cc and CMakeLists.txt from the above link and put them into to a folder called listener at your home directory. Compile the example:

        cd ~/listener
        mkdir build
        cd build
        cmake ..
        make


The example program ''listener'' should be built in the build directory. When you run ''listener'' with gazebo running, it will output something like the following:

        # Starting in ~/listener/build/
        ./listener
        Msg Waiting for master
        Msg Connected to gazebo master @ http://127.0.0.1:11345
        Msg Publicized address: 192.168.1.67
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

### Explanation of the code ###

Load gazebo and run the transport system

        gazebo::load(_argc, _argv);
        gazebo::run();

Next create a Node, which provides functions to create publishers and subscribers.

        gazebo::transport::NodePtr node(new gazebo::transport::Node());
        node->Init();

Create a subscriber on the ''world_stats'' topic. Gazebo publishes a stream of stats on this topic.

        gazebo::transport::SubscriberPtr sub = node->Subscribe("~/world_stats", cb);

You'll need to create a callback function that will print the message to the console, which we called cb in the previous line.

        void cb(ConstWorldStatisticsPtr &_msg)
        {
          std::cout << _msg->DebugString();
        }

At this point you'll have to create a wait loop while messages come in, or do some other processing. Here is simple wait loop.

        while (true)
          gazebo::common::Time::MSleep(10);

Once you're done, finalize the transport system

        gazebo::transport::fini();
