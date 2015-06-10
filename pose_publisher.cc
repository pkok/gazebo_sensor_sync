#include <gazebo/gazebo.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <gazebo/plugins/ImuSensorPlugin.hh>

namespace gazebo
{
  class PosePublisher : public ModelPlugin
  {
    public:
      virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
      {
        _model->GetWorld()->PublishModelPose(_model);
      }
  /*
      virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
      {
        //if (!gazebo::client::setup(0, (char **) NULL)) {
        //  gzerr << "gazebo::client::setup failed: could not set up a client\n";
        //}
        //else {
        this->node.reset(new transport::Node());
        std::string name = "~/" + _parent->GetName() + "/pose";
        this->pose_pub = node->Advertise<msgs::Pose>(name);
          //pose_pub->WaitForConnection();
        //}
      }

      virtual void OnUpdate(sensors::ImuSensorPtr _sensor)
      {
        msgs::Pose msg;
        msgs::Set(&msg, this->GetWorldPose());
        gzwarn << msg.position().x() << ", " << msg.position().y() << ", " << msg.position().z() << " | " << msg.orientation().x() << ", " << msg.orientation().y() << ", " << msg.orientation().z() << msg.orientation().w() << "\n";
        pose_pub->Publish(msg);
      }

    private:
      transport::NodePtr node;
      transport::PublisherPtr pose_pub;
  */
  };

  GZ_REGISTER_MODEL_PLUGIN(PosePublisher)
}
