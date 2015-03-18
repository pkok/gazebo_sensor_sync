#include "gazebo/gazebo.hh"
#include "gazebo/sensors/sensors.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"

#include <thread>

template <typename MessageType>
class Synchronizer
{
  public: Synchronizer(const gazebo::sensors::SensorPtr& sensor,
                       const std::string& topic)
          : _sensor(sensor),
            _topic(topic)
  {}

  public: void Subscribe()
  {
    gazebo::setupClient(0, (char **) NULL);
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();
    gazebo::transport::SubscriberPtr sub = node->Subscribe(_topic, &Synchronizer<MessageType>::SensorUpdate, this);
  }

  private: void SensorUpdate(const boost::shared_ptr<MessageType const>&)
  {
    _sensor->Update(true);
  }

  private: gazebo::sensors::SensorPtr _sensor;
  private: std::string _topic;
};


namespace gazebo
{
  class SensorSync : public SensorPlugin
  {
    public: SensorSync()
            : SensorPlugin()
    {}

    public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
    {
      gzwarn << "Loading SensorSync\n";
      SensorPlugin::Load(_parent, _sdf);
      _parent->SetUpdateRate(0.0);

      if (_sdf->HasElement("trigger_topic")) {
        //sdf::ElementPtr sdf_trigger_topic = _sdf->GetElement("trigger_topic");
        //trigger_topic = sdf_exposure_time->GetValue()->GetAsString();
        trigger_topic = _sdf->GetElement("trigger_topic")->GetValue()->GetAsString();
      }
      else {
        // raise some Gazebo error
      }

      // OnUpdate (and similar functions) are ommitted on purpose.
      
      // Generate a thread for the listener
      std::string sensor_type = _parent->GetType();

#define TRY_SYNCHRONIZER(SENSOR_TYPE, MESSAGE_TYPE) \
  if (sensor_type == SENSOR_TYPE) { \
    Synchronizer<MESSAGE_TYPE> syncer(_parent, trigger_topic); \
    std::thread t(&Synchronizer<MESSAGE_TYPE>::Subscribe, syncer); \
  }
      TRY_SYNCHRONIZER("imu", msgs::IMU)
      else TRY_SYNCHRONIZER("camera", msgs::ImageStamped)
      else TRY_SYNCHRONIZER("contact", msgs::Contact)
      else TRY_SYNCHRONIZER("gps", msgs::GPS)
      else TRY_SYNCHRONIZER("ray", msgs::RaySensor)
      else TRY_SYNCHRONIZER("sonar", msgs::Sonar)
      else TRY_SYNCHRONIZER("force_torque", msgs::ForceTorque)
#undef TRY_SYNCHRONIZER
    }

    protected: std::string trigger_topic;
  };

  GZ_REGISTER_SENSOR_PLUGIN(SensorSync)
}
