#include <iostream>

#include "publisher_string.hpp"

#include <std_msgs/String.h>

namespace alros
{
namespace publisher
{

StringPublisher::StringPublisher( const std::string& name, const std::string& topic ):
    BasePublisher( name, topic )
{
}

StringPublisher::StringPublisher( const std::string& name, const std::string& topic, ros::NodeHandle& nh ):
  BasePublisher( name, topic )
{
  reset( nh );
}

void StringPublisher::publish()
{
  static std_msgs::String m;
  m.data = "test_string_data";
  pub_.publish( m );
  std::cout << name() << " is publishing " << m.data << std::endl;
}

void StringPublisher::reset( ros::NodeHandle& nh)
{
  pub_ = nh.advertise<std_msgs::String>( topic_, 10 );
  is_initialized_ = true;
  std::cout << name() << " is resetting" << std::endl;
}


} //publisher
} // alros
