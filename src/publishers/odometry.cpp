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

#include <iostream>

#include <tf/transform_datatypes.h>

#include "boost/filesystem.hpp"
#include "odometry.hpp"

namespace alros
{
namespace publisher
{

OdometryPublisher::OdometryPublisher( const std::string& name, const std::string& topic, float frequency, qi::SessionPtr& session ):
  BasePublisher( name, topic, frequency, session ),
  p_motion_( session->service("ALMotion") )
{}

void OdometryPublisher::publish()
{
  std::vector<float> al_odometry_data = p_motion_.call<std::vector<float> >( "getPosition", "Torso", 1, true );
  const ros::Time& stamp = ros::Time::now();
  const float& odomX =  al_odometry_data[0];
  const float& odomY =  al_odometry_data[1];
  const float& odomZ =  al_odometry_data[2];
  const float& odomWX =  al_odometry_data[3];
  const float& odomWY =  al_odometry_data[4];
  const float& odomWZ =  al_odometry_data[5];
  //since all odometry is 6DOF we'll need a quaternion created from yaw
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw( odomWX, odomWY, odomWZ );

  /**
   * ODOMETRY FRAME
   */
  msg_tf_odom_.header.stamp = stamp;

  msg_tf_odom_.transform.translation.x = odomX;
  msg_tf_odom_.transform.translation.y = odomY;
  msg_tf_odom_.transform.translation.z = odomZ;
  msg_tf_odom_.transform.rotation = odom_quat;

  tf_br_.sendTransform( msg_tf_odom_ );

  /**
   * ODOMETRY MESSAGE
   */
  // we can improve this with a velocity computation as well
  // with this we simply set the velocity as the diff between (odomX - msg_odom_.pose.pose.position.x)/dt
//  msg_nav_odom_.header.stamp = msg_tf_odom_.header.stamp;
//
//  msg_nav_odom_.pose.pose.position.x = odomX;
//  msg_nav_odom_.pose.pose.position.y = odomY;
//  msg_nav_odom_.pose.pose.position.z = 0;
//  msg_nav_odom_.pose.pose.orientation = odom_quat;
//  // fill in velocity components here
//  pub_odom_.publish(msg_nav_odom_);
}

void OdometryPublisher::reset( ros::NodeHandle& nh )
{
  pub_odom_ = nh.advertise<nav_msgs::Odometry>( topic(), 10 );

  msg_tf_odom_.header.frame_id = "odom";
  msg_tf_odom_.child_frame_id = "base_link";

  // pre-fill odometry message
  msg_nav_odom_.header.frame_id = "odom";
  msg_nav_odom_.child_frame_id = "base_footprint";

  is_initialized_ = true;
}

bool OdometryPublisher::isSubscribed() const
{
  // assume JS and TF as essential, so publish always
  return true;
}


} //publisher
} // alros
