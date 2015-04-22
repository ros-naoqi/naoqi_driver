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

void JointStateRecorder::writeDump()
{
  boost::mutex::scoped_lock lock_write_buffer( mutex_ );
  std::list< std::vector<geometry_msgs::TransformStamped> >::iterator it_tf;
  for (it_tf = bufferTF_.begin(); it_tf != bufferTF_.end(); it_tf++)
  {
    gr_->write("/tf", *it_tf);
  }
  for (std::list<sensor_msgs::JointState>::iterator it_js = bufferJoinState_.begin();
       it_js != bufferJoinState_.end(); it_js++)
  {
    if (!it_js->header.stamp.isZero()) {
      gr_->write(topic_, *it_js, it_js->header.stamp);
    }
    else {
      gr_->write(topic_, *it_js);
    }
  }
}

void JointStateRecorder::reset(boost::shared_ptr<GlobalRecorder> gr, float frequency)
{
  gr_ = gr;
  buffer_size_ = static_cast<size_t>(10*frequency);
  bufferJoinState_.resize(buffer_size_);
  bufferTF_.resize(buffer_size_);
  is_initialized_ = true;
}

void JointStateRecorder::bufferize( const sensor_msgs::JointState& js_msg,
                const std::vector<geometry_msgs::TransformStamped>& tf_transforms )
{
  boost::mutex::scoped_lock lock_bufferize( mutex_ );
  bufferJoinState_.pop_front();
  bufferTF_.pop_front();
  bufferJoinState_.push_back(js_msg);
  bufferTF_.push_back(tf_transforms);
}

} //publisher
} // alros
