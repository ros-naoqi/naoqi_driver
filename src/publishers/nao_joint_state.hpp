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

#ifndef NAO_JOINT_STATES_PUBLISHER_HPP
#define NAO_JOINT_STATES_PUBLISHER_HPP

#include "joint_state.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

namespace alros
{
namespace publisher
{

class NaoJointStatePublisher : public JointStatePublisher
{

public:
  NaoJointStatePublisher( const std::string& name, const std::string& topic, float frequency, qi::SessionPtr& session );

  virtual void publish();

  virtual void reset( ros::NodeHandle& nh );

private:
  boost::shared_ptr<tf2_ros::Buffer> tf_bufferPtr_;
  boost::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcasterPtr_;
  boost::shared_ptr<tf2_ros::TransformListener> tf_listenerPtr_;
}; // class

} //publisher
} // alros

#endif
