#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/plugins/ImuSensorPlugin.hh>

#include <iostream>

namespace gazebo
{
  typedef gazebo::msgs::ImageStamped ImageMsg;
  typedef gazebo::msgs::ImageStampedPtr ImageMsgPtr;
  typedef gazebo::msgs::IMU ImuMsg;
  typedef gazebo::msgs::IMUPtr ImuMsgPtr;

  class MinimalImuPlugin : public ImuSensorPlugin
  {
    public: MinimalImuPlugin()
            : ImuSensorPlugin(),
            _updated_image(false),
            _updated_imu(false)
    {}

    public: virtual void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
    {
      gzmsg << "Loading MinimalImuPlugin\n";
      ImuSensorPlugin::Load(_parent, _sdf);
      //if (_sdf->HasElement("trigger_topic")) {
      //}
      //else {
      //  gzwarn << "No trigger_topic specified in world file.\n";
      //}

      //if (_sdf->HasElement("logfile")) {
      //}
      //else {
      //  gzwarn << "No logfile specified in world file.\n";
      //}

      //gazebo::transport::NodePtr node(new gazebo::transport::Node());
      //node->Init(_parent->GetWorldName());
      //std::string _topic = "";
      ////gazebo::transport::SubscriberPtr sub = node->Subscribe(_topic, &MinimalImuPlugin::DoOtherUpdate, this, true);

    }

    //public: virtual void DoOtherUpdate(const ImageMsgPtr& message)
    //{
    //  _updated_image = true;
    //  PublishIfFullyUpdated();
    //}

    public: virtual void OnUpdate(sensors::ImuSensorPtr _sensor) 
    {
      std::cout << "Sensor updated." << std::endl;
      //gzwarn << "Sensor updated.\n";
      _updated_imu = true;
      //PublishIfFullyUpdated();
    }

    protected: virtual void PublishIfFullyUpdated()
    {
      if (_updated_image && _updated_imu) {
        gzmsg << "Hurray, you can publish stuff!\n";
        _updated_image = false;
        _updated_imu = false;
      }
      else {
        if (_updated_image) {
          gzmsg << "You can only publish the image\n";
        }
        else if (_updated_imu) {
          gzmsg << "You can only publish the IMU data\n";
        }
        else {
          gzmsg << "Nothing to publish.\n";
        }
      }
    }

    protected: bool _updated_image;
    protected: bool _updated_imu;
  };
  
  GZ_REGISTER_SENSOR_PLUGIN(MinimalImuPlugin)
}
