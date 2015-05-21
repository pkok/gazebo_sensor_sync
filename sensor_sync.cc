/* Almost synchronize two sensor updates, and store results in file.
 * 
 * A SensorPlugin registers itself to a topic by passing it a <trigger_topic>
 * in the Gazebo .world file.  Whenever messages appear on that topic, that
 * message and the current readings of the SensorPlugin's sensor will be stored
 * to a <logfile>.
 */
#include "protobuf_helper.hh"

#include <gazebo/gazebo.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <gazebo/plugins/ImuSensorPlugin.hh>

#include <google/protobuf/io/zero_copy_stream_impl.h>

#include <thread>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

/** Listens to a topic and lets a sensor generate output based on the topic's publishing frequency.  */
template <typename MessageType>
class Synchronizer
{
  public: typedef boost::shared_ptr<const MessageType> MessagePtr;

  public: Synchronizer(const gazebo::sensors::SensorPtr& sensor,
                       const std::string& topic,
                       const MessagePtr& message)
          : _sensor(sensor),
            _topic(topic),
            _message(const_cast<MessagePtr&>(message))
  {}

  /** Starts the listener. */
  public: bool Subscribe(const std::string& _worldname)
  {
    /*
    if (!gazebo::client::setup(0, (char **) NULL)) {
      gzerr << "gazebo::client::setup failed: could not make a listener node.\n";
      return false;
    }
    */
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init(_worldname);
    gazebo::transport::SubscriberPtr sub = node->Subscribe(_topic, &Synchronizer<MessageType>::SensorUpdate, this, true);
    return true;
  }


  private: void SensorUpdate(const MessagePtr& message)
  {
    gzmsg << "Check 0\n";
    //_sensor->Update(true);
    gzmsg << "Check 0.5\n";
    _message = message;
    gzmsg << "Check 1\n";
  }

  private: gazebo::sensors::SensorPtr _sensor;
  private: std::string _topic;
  private: MessagePtr& _message;
};

typedef Synchronizer<gazebo::msgs::ImageStamped> ImuCamSyncer;


namespace gazebo
{
  class ImuSensorSync : public ImuSensorPlugin
  {
    public: ImuSensorSync()
            : ImuSensorPlugin() 
    {}

    public: virtual void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
    {
      gzmsg << "Loading ImuSensorSync\n";
      //_parent->SetUpdateRate(60.0);
      //_parent->SetUpdateRate(0.0);

      //if (_sdf->HasElement("trigger_topic")) {
      //  trigger_topic = _sdf->GetElement("trigger_topic")->GetValue()->GetAsString();
      //}
      //else {
      //  throw gazebo::common::Exception(__FILE__, __LINE__, "No <trigger_topic> found, but required for plugin ImuSensorSync. Forgot to declare this tag in the .world file?");
      //}

      //if (_sdf->HasElement("logfile")) {
      //  std::string logfile_path = _sdf->GetElement("logfile")->GetValue()->GetAsString();
      //  int logfile_descriptor = open(logfile_path.c_str(), O_WRONLY | O_TRUNC);
      //  logfile = new google::protobuf::io::FileOutputStream(logfile_descriptor);
      //  logfile->SetCloseOnDelete(true);
      //  //logfile->open(logfile_path, std::ios::out | std::ios::binary | std::ios::trunc);
      //}
      //else {
      //  throw gazebo::common::Exception(__FILE__, __LINE__, "No <logfile> found, but required for plugin ImuSensorSync. Forgot to declare this tag in the .world file?");
      //}

      //// Generate a thread for the listener
      //std::string sensor_type = _parent->GetType();
      //ImuCamSyncer syncer(_parent, trigger_topic, image);
      //syncer.Subscribe(_parent->GetWorldName());
      //// I don't think threading is necessary, not 100% sure yet.
      ////std::thread t(&ImuCamSyncer::Subscribe, syncer);
      ////t.detach();

      ImuSensorPlugin::Load(_parent, _sdf);
    }

    //public: virtual void OnUpdate(const sensors::ImuSensorPtr& imu)
    public: virtual void OnUpdate(sensors::ImuSensorPtr _sensor) 
    {
      gzerr << "Writing data to logfile\n";
      //// Ground truth
      //gazebo::msgs::Pose pose;
      //gazebo::math::Pose p = this->parentSensor->GetPose();
      //pose.mutable_position()->set_x(p.pos.x);
      //pose.mutable_position()->set_y(p.pos.y);
      //pose.mutable_position()->set_z(p.pos.z);
      //pose.mutable_orientation()->set_w(p.rot.w);
      //pose.mutable_orientation()->set_x(p.rot.x);
      //pose.mutable_orientation()->set_y(p.rot.y);
      //pose.mutable_orientation()->set_z(p.rot.z);
      //writeDelimitedTo(pose, logfile);
      //// Synchronized image data
      //writeDelimitedTo(*image, logfile);
      //// IMU data
      //writeDelimitedTo(_sensor->GetImuMessage(), logfile);
      //logfile->Flush();
    }

    protected: std::string trigger_topic;
    protected: ConstImageStampedPtr image;
    protected: google::protobuf::io::FileOutputStream* logfile;
  };

  GZ_REGISTER_SENSOR_PLUGIN(ImuSensorSync)
}

