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
namespace recorder
{

JointStateRecorder::JointStateRecorder( const std::string& topic, float buffer_frequency ):
  topic_( topic ),
  buffer_duration_(helpers::recorder::bufferDefaultDuration),
  is_initialized_( false ),
  is_subscribed_( false ),
  buffer_frequency_(buffer_frequency),
  counter_(1)
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

void JointStateRecorder::writeDump(const ros::Time& time)
{
  boost::mutex::scoped_lock lock_write_buffer( mutex_ );
  boost::circular_buffer< std::vector<geometry_msgs::TransformStamped> >::iterator it_tf;
  for (it_tf = bufferTF_.begin(); it_tf != bufferTF_.end(); it_tf++)
  {
    gr_->write("/tf", *it_tf);
  }
  for (boost::circular_buffer<sensor_msgs::JointState>::iterator it_js = bufferJoinState_.begin();
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

void JointStateRecorder::reset(boost::shared_ptr<GlobalRecorder> gr, float conv_frequency)
{
  gr_ = gr;
  conv_frequency_ = conv_frequency;
  if (buffer_frequency_ != 0)
  {
    max_counter_ = static_cast<int>(conv_frequency/buffer_frequency_);
    buffer_size_ = static_cast<size_t>(buffer_duration_*(conv_frequency/max_counter_));
  }
  else
  {
    max_counter_ = 1;
    buffer_size_ = static_cast<size_t>(buffer_duration_*conv_frequency);
  }
  bufferJoinState_.resize(buffer_size_);
  bufferTF_.resize(buffer_size_);
  is_initialized_ = true;
}

void JointStateRecorder::bufferize( const sensor_msgs::JointState& js_msg,
                const std::vector<geometry_msgs::TransformStamped>& tf_transforms )
{
  boost::mutex::scoped_lock lock_bufferize( mutex_ );
  if (counter_ < max_counter_)
  {
    counter_++;
  }
  else
  {
    counter_ = 1;
    bufferJoinState_.push_back(js_msg);
    bufferTF_.push_back(tf_transforms);
  }
}

void JointStateRecorder::setBufferDuration(float duration)
{
  boost::mutex::scoped_lock lock_bufferize( mutex_ );
  buffer_size_ = static_cast<size_t>(duration*(conv_frequency_/max_counter_));
  buffer_duration_ = duration;
  bufferJoinState_.set_capacity(buffer_size_);
  bufferTF_.set_capacity(buffer_size_);
}

} //publisher
} // naoqi
