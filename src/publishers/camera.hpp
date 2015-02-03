#ifndef PUBLISHER_CAMERA_HPP
#define PUBLISHER_CAMERA_HPP

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <qi/anyobject.hpp>

#include "publisher_base.hpp"

namespace alros
{
namespace publisher
{

class CameraPublisher : public BasePublisher<CameraPublisher>
{
public:
  CameraPublisher( const std::string& name, const std::string& topic, float frequency, const qi::AnyObject& p_video);

  void publish();

  void reset( ros::NodeHandle& nh );

  inline bool isSubscribed() const
  {
    if (is_initialized_ == false) return false;
    return pub_.getNumSubscribers() > 0;
  }

private:
  //image_transport::ImageTransport it_;
  image_transport::Publisher pub_;

  qi::AnyObject p_video_;
  sensor_msgs::ImagePtr msg_;
  std::string handle_;
};

} //publisher
} //alros


#endif
