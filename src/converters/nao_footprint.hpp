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

#ifndef NAO_FOOTPRINT_HPP
#define NAO_FOOTPRINT_HPP

/*
* ROS includes
*/
#include <tf2/LinearMath/Transform.h>
#include <geometry_msgs/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

/*
* loca includes
*/
#include "../helpers/transform_helpers.hpp"

namespace naoqi
{
namespace converter
{
namespace nao
{

inline void addBaseFootprint( boost::shared_ptr<tf2_ros::Buffer> tf2_buffer, std::vector<geometry_msgs::TransformStamped>& tf_transforms, const ros::Time& time )
{
  bool canTransform = tf2_buffer->canTransform("odom", "l_sole", time, ros::Duration(0.1) );
  if (!canTransform)
  {
    ROS_ERROR_STREAM("Do not calculate NAO Footprint, no transform possible " << time);
    return;
  }

  geometry_msgs::TransformStamped tf_odom_to_base, tf_odom_to_left_foot, tf_odom_to_right_foot;
  try {
    // TRANSFORM THEM DIRECTLY INTO TRANSFORM
    tf_odom_to_left_foot  = tf2_buffer->lookupTransform("odom", "l_sole",    time );
    tf_odom_to_right_foot = tf2_buffer->lookupTransform("odom", "r_sole",    time );
    tf_odom_to_base       = tf2_buffer->lookupTransform("odom", "base_link", time );
  } catch (const tf::TransformException& ex){
    ROS_ERROR("NAO Footprint error %s",ex.what());
    return ;
  }
  // middle of both feet
  // z = fix to the lowest foot
  tf2::Vector3 new_origin(
      float(tf_odom_to_right_foot.transform.translation.x + tf_odom_to_left_foot.transform.translation.x)/2.0,
      float(tf_odom_to_right_foot.transform.translation.y + tf_odom_to_left_foot.transform.translation.y)/2.0,
      std::min(tf_odom_to_left_foot.transform.translation.z, tf_odom_to_right_foot.transform.translation.z)
      );

  // adjust yaw according to torso orientation, all other angles 0 (= in z-plane)
  double yaw = helpers::transform::getYaw( tf_odom_to_base.transform ) ;
  tf2::Quaternion new_q;
  new_q.setRPY(0.0f, 0.0f, yaw);
  tf2::Transform tf_odom_to_footprint( new_q, new_origin);

  // way too complicated here, there should be proper conversions!
  tf2::Quaternion q( tf_odom_to_base.transform.rotation.x,
                     tf_odom_to_base.transform.rotation.y,
                     tf_odom_to_base.transform.rotation.z,
                     tf_odom_to_base.transform.rotation.w
      );
  tf2::Vector3 r( tf_odom_to_base.transform.translation.x,
                  tf_odom_to_base.transform.translation.y,
                  tf_odom_to_base.transform.translation.z
      );
  tf2::Transform tf_odom_to_base_conv( q,r);
  //tf2::Transform tf_odom_to_base_conv;
  //tf2::convert( tf_odom_to_base.transform, tf_odom_to_base_conv );
  tf2::Transform tf_base_to_footprint = tf_odom_to_base_conv.inverse() * tf_odom_to_footprint;

  // convert it back to geometry_msgs
  geometry_msgs::TransformStamped message;
  //message.transform = tf2::toMsg(tf_base_to_footprint);
  message.header.stamp = time;
  message.header.frame_id = "base_link";
  message.child_frame_id = "base_footprint";

  message.transform.rotation.x = tf_base_to_footprint.getRotation().x();
  message.transform.rotation.y = tf_base_to_footprint.getRotation().y();
  message.transform.rotation.z = tf_base_to_footprint.getRotation().z();
  message.transform.rotation.w = tf_base_to_footprint.getRotation().w();
  message.transform.translation.x = tf_base_to_footprint.getOrigin().x();
  message.transform.translation.y = tf_base_to_footprint.getOrigin().y();
  message.transform.translation.z = tf_base_to_footprint.getOrigin().z();

  //tf::transformTFToMsg( tf_base_to_footprint, message.transform);
  // publish transform with parent m_baseFrameId and new child m_baseFootPrintID
  // i.e. transform from m_baseFrameId to m_baseFootPrintID
  tf2_buffer->setTransform( message, "naoqiconverter", false );
  tf_transforms.push_back( message );
}

} // nao
} // converter
} // naoqi

#endif
