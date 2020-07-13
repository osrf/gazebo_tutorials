# Profiler

Gazebo 9 and 11 are using [Ignition Common Profiler](https://ignitionrobotics.org/api/common/3.5/profiler.html) to check and visualize the performance of the different threads, functions and methods inside Gazebo.

You must need to compile Gazebo 9 or 11 with the CMake flag `-DENALE_PROFILER=1`. For example, if you are using `colcon` to compile Gazebo:

```bash
colcon build --cmake-args -DENABLE_PROFILER=1
```

## How can I add the profiler to my own Gazebo plugins?

If you want to add the profiler to your own Gazebo plugins you need to follow this steps:

 - Include the header
    - If you are using Gazebo 11
```cpp
 #include <ignition/common/Profiler.hh>
```
    - If you are using Gazebo 9
```cpp
 #include <gazebo/util/Profiler.hh>
```

 - Then in the update method or other periodic method add the MACROS:
```cpp
IGN_PROFILE("WindPlugin::OnUpdate");
IGN_PROFILE_BEGIN("Update");
...
IGN_PROFILE_END();
```
### Sensor classes
Classes that inherit from `Sensor` should have the MACROS in the `UpdateImpl` method. This method is called periodically and it's in charge of generating the data and publish it in ignition transport.
### Plugins
Classes that inherit from `Plugins` and have some connection with the `WorldUpdate` method, for example like the `WindPlugin`, where you can find the folliwing call to `ConnectWorldUpdateBegin`. This will call the `OnUpdate` method when the world is updated periodically.
```cpp
this->dataPtr->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&WindPlugin::OnUpdate, this));
```
 - Finally link againts the profiler library.
    - If you are using Gazebo 11
```cpp
target_link_libraries(<your plugin name>
    ...
    ${IGNITION-COMMON_LIBRARIES}
)
```
    - If you are using Gazebo 9
```cpp
target_link_libraries(<your plugin name>
    ...
    gazebo_util
)
```

## How do I run and see the profiler results for Gazebo?

Use a launcher script (Linux and macOS)

```
ign_remotery_vis
```

If you are successful, you should see the profiler output in a browser. You should visualize three plots corresponding to the diferrent theads: `gzserver`, `[Ode,Bullet,Symbody,Dart]Physics` and `SensorManager`.

The following image shows the Sensor manager timeline. In this case we have 5 different sensors: `gps`, `imu`, `magnetometer`, `altimeter` and `contact`. You can see which is the duration of the sensor manager loop (50ms) and how long that it takes to update all the sensors (around 0.177ms), the rest of the time the thead will be sleeping. You can see the update time of each sensor, all the sensors have two subtask: generate de data and then publish this data in ignition trasnport (fill the data structur and publish it).

[[file:files/SensorManagerLoop.png|600px]]

In the following two images you can see the evolution of `OdePhysics` and the `gzserver` loops.

[[file:files/Remotery_OdePhysics.png|600px]]

[[file:files/gzserverLoop.png|600px]]
