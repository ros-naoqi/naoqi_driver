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

/*
* LOCAL includes
*/
#include "joint_state.hpp"

namespace naoqi
{
namespace publisher
{

JointStatePublisher::JointStatePublisher( const std::string& topic ):
  topic_( topic ),
  is_initialized_( false )
{}

void JointStatePublisher::publish( const sensor_msgs::JointState& js_msg,
                                   const std::vector<geometry_msgs::TransformStamped>& tf_transforms )
{
  pub_joint_states_.publish( js_msg );

  /**
   * ROBOT STATE PUBLISHER
   */
  tf_broadcasterPtr_->sendTransform(tf_transforms);
}


void JointStatePublisher::reset( ros::NodeHandle& nh )
{
  pub_joint_states_ = nh.advertise<sensor_msgs::JointState>( topic_, 10 );

  tf_broadcasterPtr_ = boost::make_shared<tf2_ros::TransformBroadcaster>();

  is_initialized_ = true;
}

bool JointStatePublisher::isSubscribed() const
{
  // assume JS and TF as essential, so publish always
  return true;
}

} //publisher
} // naoqi
