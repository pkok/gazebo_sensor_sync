#include "syncbuffer.hh"

#include <gazebo/gazebo.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <iostream>

typedef gazebo::msgs::IMU ImuType;
typedef gazebo::msgs::ImageStamped ImageType;
typedef gazebo::msgs::PosesStamped PoseType;

typedef SyncBuffer<ImuType, ImageType, PoseType> SyncedBuffer;

int main(int, char**)
{
  if (!gazebo::client::setup(0, (char **) NULL)) {
    std::cerr << "gazebo::client::setup failed: could not set up a client\n";
    return EXIT_FAILURE;
  }

  SyncedBuffer sync_buffer("/tmp/my.protobuf");

  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();
  gazebo::transport::SubscriberPtr imu_sub 
    = node->Subscribe(std::string("~/camimu/base_link/imu/imu"),
                      &SyncedBuffer::buffer<0, ImuType>, 
                      &sync_buffer, true);
  gazebo::transport::SubscriberPtr cam_sub 
    = node->Subscribe(std::string("~/camimu/base_link/camera/image"),
                      &SyncedBuffer::buffer<1, ImageType>,
                      &sync_buffer, true);
  gazebo::transport::SubscriberPtr pose_sub 
    = node->Subscribe(std::string("~/pose/local/info"),
                      &SyncedBuffer::buffer<2, PoseType>,
                      &sync_buffer, true);

  while (true)
    gazebo::common::Time::MSleep(10);
  /*
  */

  gazebo::client::shutdown();
  return EXIT_SUCCESS;
}
