#include <iostream>

#include "publisher_joint_state.hpp"

namespace alros
{
namespace publisher
{

JointStatePublisher::JointStatePublisher( const std::string& name, const std::string& topic, qi::AnyObject p_motion ):
  BasePublisher( name, topic ),
  p_motion_( p_motion )
{}

JointStatePublisher::JointStatePublisher( const std::string& name, const std::string& topic, qi::AnyObject p_motion, ros::NodeHandle& nh ):
  BasePublisher( name, topic ),
  p_motion_( p_motion )
{
  reset( nh );
}

void JointStatePublisher::publish()
{
  std::vector<float> floats = p_motion_.call<std::vector<float> >("getAngles", "JointActuators", true );
  msg_.position = std::vector<double>( floats.begin(), floats.end() );
  msg_.header.stamp = ros::Time::now();
  std::cout << name() << " is publishing " << std::endl;
  pub_.publish( msg_ );
}

void JointStatePublisher::reset( ros::NodeHandle& nh )
{
  pub_ = nh.advertise<sensor_msgs::JointState>( topic_, 10 );

  msg_.name = p_motion_.call<std::vector<std::string> >("getBodyNames", "JointActuators" );

  is_initialized_ = true;
  std::cout << name() << " is resetting" << std::endl;
}

} //publisher
} // alros
