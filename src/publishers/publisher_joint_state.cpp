#include <iostream>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>

#include <robot_state_publisher/robot_state_publisher.h>

#include "publisher_joint_state.hpp"

namespace alros
{
namespace publisher
{

JointStatePublisher::JointStatePublisher( const std::string& name, const std::string& topic, float frequency, qi::AnyObject& p_motion ):
  BasePublisher( name, topic, frequency ),
  p_motion_( p_motion )
{}

void JointStatePublisher::publish()
{
  std::vector<float> floats = p_motion_.call<std::vector<float> >("getAngles", "JointActuators", true );
  msg_.position = std::vector<double>( floats.begin(), floats.end() );
  msg_.header.stamp = ros::Time::now();
  std::cout << name() << " is publishing " << std::endl;
  pub_.publish( msg_ );

  std::map< std::string, double > joint_state_map;
  // stupid version --> change this with std::transform c++11 ??!
  std::transform( msg_.name.begin(), msg_.name.end(), msg_.position.begin(), std::inserter( joint_state_map, joint_state_map.end() ), std::make_pair<std::string, double>);

  static const std::string& jt_tf_prefix = "";
  rspPtr_->publishTransforms( joint_state_map, msg_.header.stamp, jt_tf_prefix );
  rspPtr_->publishFixedTransforms( jt_tf_prefix );

}

void JointStatePublisher::reset( ros::NodeHandle& nh )
{
  pub_ = nh.advertise<sensor_msgs::JointState>( topic_, 10 );

  // load urdf from param server (alternatively from file)
  if ( nh.hasParam("/robot_description") )
  {
    KDL::Tree tree;
    std::string robot_desc;
    nh.getParam("/robot_description", robot_desc);
    kdl_parser::treeFromString( robot_desc, tree );
    rspPtr_.reset( new robot_state_publisher::RobotStatePublisher(tree) );
  }
  else{
    std::cerr << "failed to load robot description in joint_state_publisher" << std::endl;
    is_initialized_ = false;
    return;
  }

  msg_.name = p_motion_.call<std::vector<std::string> >("getBodyNames", "JointActuators" );

  is_initialized_ = true;
  std::cout << name() << " is resetting" << std::endl;
}

bool JointStatePublisher::isSubscribed() const
{
  // assume JS and TF as essential, so publish always
  return true;
}


} //publisher
} // alros
