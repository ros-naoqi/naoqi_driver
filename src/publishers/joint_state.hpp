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
#include <geometry_msgs/Transform.h>
#include <robot_state_publisher/robot_state_publisher.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>

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
  JointStatePublisher( const std::string& name, const std::string& topic, float frequency, qi::SessionPtr& session );

  virtual void publish();

  virtual void reset( ros::NodeHandle& nh );

  virtual bool isSubscribed() const;

  boost::shared_ptr<tf2_ros::Buffer> getTF2Buffer();

protected:
  sensor_msgs::JointState msg_joint_states_;

  /** blatently copied from robot state publisher */
  void addChildren(const KDL::SegmentMap::const_iterator segment);
  std::map<std::string, robot_state_publisher::SegmentPair> segments_, segments_fixed_;

private:

  void setTransforms(const std::map<std::string, double>& joint_positions, const ros::Time& time, const std::string& tf_prefix);
  void setFixedTransforms(const std::string& tf_prefix, const ros::Time& time);

  qi::AnyObject p_motion_;

  tf2_ros::TransformBroadcaster tf_broadcaster_;

  /** initialize separate publishers for js and odom */
  ros::Publisher pub_joint_states_;

  /** tf2 buffer that is filled with transform data if other persons own it */
  boost::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
}; // class

} //publisher
} // alros

#endif
