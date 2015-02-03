#ifndef JOINT_STATES_PUBLISHER_HPP
#define JOINT_STATES_PUBLISHER_HPP

/**
* ROS includes
*/
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <robot_state_publisher/robot_state_publisher.h>
/**
* ALDEBARAN includes
*/
#include <qi/anyobject.hpp>

#include <vector>

#include "publisher_base.hpp"

namespace alros
{
namespace publisher
{


class JointStatePublisher : public BasePublisher<JointStatePublisher>
{

public:
  JointStatePublisher( const std::string& name, const std::string& topic, float frequency, qi::AnyObject& p_motion );

  void publish();

  void reset( ros::NodeHandle& nh );

  bool isSubscribed() const;

private:
  qi::AnyObject p_motion_;
  sensor_msgs::JointState msg_;
  boost::shared_ptr<robot_state_publisher::RobotStatePublisher> rspPtr_;
}; // class

} //publisher
} // alros

#endif
