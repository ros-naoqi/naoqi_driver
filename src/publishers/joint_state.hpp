/*
 * Copyright 2015 Aldebaran
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef JOINT_STATES_PUBLISHER_HPP
#define JOINT_STATES_PUBLISHER_HPP

/**
* ROS includes
*/
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <robot_state_publisher/robot_state_publisher.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Transform.h>
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

  /** initialize separate publishers for js and odom */
  ros::Publisher pub_joint_states_;
  ros::Publisher pub_odom_;
  /* initialize the broadcaster for publishing the odom frame */
  tf::TransformBroadcaster tf_br_;

  sensor_msgs::JointState msg_joint_states_;
  nav_msgs::Odometry msg_nav_odom_;
  geometry_msgs::TransformStamped msg_tf_odom_;
  boost::shared_ptr<robot_state_publisher::RobotStatePublisher> rspPtr_;
}; // class

} //publisher
} // alros

#endif
