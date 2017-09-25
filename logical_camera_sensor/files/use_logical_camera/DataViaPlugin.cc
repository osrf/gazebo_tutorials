#include <iostream>

#include <boost/bind.hpp>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>

namespace gazebo
{
  class DataViaPlugin : public SensorPlugin
  {
    public: DataViaPlugin() : SensorPlugin()
    {
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&DataViaPlugin::OnUpdate, this, _1));
    }

    public: void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
    {
      printf("On Load\n");
      this->sensor = std::static_pointer_cast<sensors::LogicalCameraSensor>(
          _sensor);
    }

    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
      // This gets the latest sensor data
      msgs::LogicalCameraImage sensorData = this->sensor->Image();

      for (auto &model : sensorData.model())
      {
        bool isNew = true;
        for (auto &old_model : this->lastData.model())
        {
          if (model.name() == old_model.name())
          {
            isNew = false;
            break;
          }
        }
        if (isNew)
          std::cout << "I see [" << model.name() << "]\n";
      }
      this->lastData = sensorData;
    }

    private: sensors::LogicalCameraSensorPtr sensor;

    private: msgs::LogicalCameraImage lastData;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };
  GZ_REGISTER_SENSOR_PLUGIN(DataViaPlugin)
}
