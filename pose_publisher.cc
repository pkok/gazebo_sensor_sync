/* Publish the pose of any model. */
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
  };

  GZ_REGISTER_MODEL_PLUGIN(PosePublisher)
}
