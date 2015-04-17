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

/**
* LOCAL includes
*/
#include "joint_state.hpp"

namespace alros
{
namespace recorder
{

JointStateRecorder::JointStateRecorder( const std::string& topic ):
  topic_( topic ),
  is_initialized_( false ),
  is_subscribed_( false )
{}

void JointStateRecorder::write( const sensor_msgs::JointState& js_msg,
                                const std::vector<geometry_msgs::TransformStamped>& tf_transforms )
{
  if (!js_msg.header.stamp.isZero()) {
    gr_->write(topic_, js_msg, js_msg.header.stamp);
  }
  else {
    gr_->write(topic_, js_msg);
  }
  gr_->write("/tf", tf_transforms);
}

void JointStateRecorder::reset(boost::shared_ptr<GlobalRecorder> gr)
{
  gr_ = gr;
  is_initialized_ = true;
}

} //publisher
} // alros
