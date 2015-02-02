#ifndef PUBLISHER_CAMERA_HPP
#define PUBLISHER_CAMERA_HPP

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <qi/anyobject.hpp>

#include "publisher_base_image.hpp"

namespace alros
{
namespace publisher
{

class CameraPublisher : public ImageBasePublisher<CameraPublisher>
{
public:
  CameraPublisher( const std::string& name, const std::string& topic, const qi::AnyObject& p_video);

  void publish();

  void reset( ros::NodeHandle& nh );


private:
  qi::AnyObject p_video_;
  sensor_msgs::ImagePtr msg_;
  std::string handle_;
};

} //publisher
} //alros


#endif
