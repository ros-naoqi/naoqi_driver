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

#include "nao_joint_state.hpp"
#include <tf2/LinearMath/Transform.h>
#include <geometry_msgs/Transform.h>

namespace alros
{
namespace publisher
{

NaoJointStatePublisher::NaoJointStatePublisher( const std::string& name, const std::string& topic, float frequency, qi::SessionPtr& session ):
  JointStatePublisher( name, topic, frequency, session ),
  tf2_buffer_(getTF2Buffer())
{}

void NaoJointStatePublisher::publish()
{
  JointStatePublisher::publish();

  // use msg_joint_states_.header.stamp for getting time stamp
  // The 0.1 is totally arbitrary: it's due to the fact that odom and the rest of tf are published at
  // different timestamps
  ros::Time time = msg_joint_states_.header.stamp - ros::Duration(0.1);
  geometry_msgs::TransformStamped tf_odom_to_base, tf_odom_to_left_foot, tf_odom_to_right_foot;

  bool canTransform = tf2_buffer_->canTransform("odom", "l_sole", time, ros::Duration(0.1) );
  try {
    // TRANSFORM THEM DIRECTLY INTO TRANSFORM
    tf_odom_to_left_foot  = tf2_buffer_->lookupTransform("odom", "l_sole",    time );
    tf_odom_to_right_foot = tf2_buffer_->lookupTransform("odom", "r_sole",    time );
    tf_odom_to_base       = tf2_buffer_->lookupTransform("odom", "base_link", time );
  } catch (const tf::TransformException& ex){
    ROS_ERROR("%s",ex.what());
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
  double yaw = tf::getYaw( tf_odom_to_base.transform.rotation );

  tf2::Transform tf_odom_to_footprint( tf2::Quaternion(0.0f, 0.0f, yaw), new_origin);

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
  tf2::Transform tf_base_to_footprint = tf_odom_to_base_conv.inverse() * tf_odom_to_footprint;

  // convert it back to geometry_msgs
  geometry_msgs::TransformStamped message;
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
  tf2_buffer_->setTransform(message, "alrosconverter", false);
  tf_broadcasterPtr_->sendTransform( message );
}

void NaoJointStatePublisher::reset( ros::NodeHandle& nh )
{
  JointStatePublisher::reset(nh);

  tf_broadcasterPtr_.reset( new tf2_ros::TransformBroadcaster() );
}


} //publisher
} // alros
