#include <iostream>

#include <std_msgs/Int32.h>

#include "publisher_int.hpp"

namespace alros
{
namespace publisher
{

IntPublisher::IntPublisher( const std::string& name, const std::string& topic, float frequency ):
  BasePublisher( name, topic, frequency )
{}

IntPublisher::IntPublisher( const std::string& name, const std::string& topic, float frequency, ros::NodeHandle& nh ):
  BasePublisher( name, topic, frequency )
{
  reset( nh );
}

void IntPublisher::publish()
{
  static std_msgs::Int32 m;
  m.data++;
  std::cout << name() << " is publishing " << m.data << std::endl;
  pub_.publish( m );
}

void IntPublisher::reset( ros::NodeHandle& nh )
{
  pub_ = nh.advertise< std_msgs::Int32>( topic_, 10 );
  is_initialized_ = true;
  std::cout << name() << " is resetting" << std::endl;
}

} //publisher
} // alros
