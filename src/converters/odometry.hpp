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

#ifndef ODOMETRY_PUBLISHER_HPP
#define ODOMETRY_PUBLISHER_HPP

/*
* LOCAL includes
*/
#include "converter_base.hpp"

/*
* ROS includes
*/
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/Transform.h>
/*
* ALDEBARAN includes
*/
#include <qi/anyobject.hpp>

#include <vector>

namespace tf2_ros
{
  class Buffer;
}

namespace alros
{
namespace publisher
{


class OdometryPublisher : public BasePublisher<OdometryPublisher>
{

public:
  OdometryPublisher( const std::string& name, const std::string& topic, float frequency, qi::SessionPtr& session,
    boost::shared_ptr<tf2_ros::Buffer>& tf2_buffer );

  void publish();

  void reset( ros::NodeHandle& nh );

  bool isSubscribed() const;

private:
  qi::AnyObject p_motion_;

  /** initialize separate publishers for js and odom */
  ros::Publisher pub_odom_;
  /* initialize the broadcaster for publishing the odom frame */
  tf2_ros::TransformBroadcaster tf_br_;

  nav_msgs::Odometry msg_nav_odom_;
  geometry_msgs::TransformStamped msg_tf_odom_;
  boost::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
}; // class

} //publisher
} // alros

#endif
