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
#include "camera.hpp"

namespace naoqi
{
namespace recorder
{

CameraRecorder::CameraRecorder( const std::string& topic_, float buffer_frequency ):
  buffer_frequency_(buffer_frequency),
  buffer_duration_( helpers::recorder::bufferDefaultDuration ),
  counter_(1)
{
  topic_info_ = topic_ + "/camera_info";
  topic_img_ = topic_ + "/image_raw";
}

void CameraRecorder::write(const sensor_msgs::ImagePtr& img, const sensor_msgs::CameraInfo& camera_info)
{
  if (!img->header.stamp.isZero()) {
    gr_->write(topic_img_, *img, img->header.stamp);
  }
  else {
    gr_->write(topic_img_, *img);
  }
  if (!camera_info.header.stamp.isZero()) {
    gr_->write(topic_info_, camera_info, camera_info.header.stamp);
  }
  else {
    gr_->write(topic_info_, camera_info);
  }
}

void CameraRecorder::writeDump(const ros::Time& time)
{
  boost::mutex::scoped_lock lock_write_buffer( mutex_ );
  boost::circular_buffer< std::pair<sensor_msgs::ImagePtr, sensor_msgs::CameraInfo> >::iterator it;
  for (it = buffer_.begin(); it != buffer_.end(); it++)
  {
    if (it->first != NULL)
    {
      write(it->first, it->second);
    }
  }
}

void CameraRecorder::reset(boost::shared_ptr<GlobalRecorder> gr, float conv_frequency)
{
  gr_ = gr;
  conv_frequency_ = conv_frequency;
  max_counter_ = static_cast<int>(conv_frequency/buffer_frequency_);
  buffer_size_ = static_cast<size_t>(buffer_duration_*(conv_frequency/max_counter_));
  buffer_.resize(buffer_size_);
  is_initialized_ = true;
}

void CameraRecorder::bufferize( const sensor_msgs::ImagePtr& img, const sensor_msgs::CameraInfo& camera_info )
{
  boost::mutex::scoped_lock lock_bufferize( mutex_ );
  if (counter_ < max_counter_)
  {
    counter_++;
  }
  else
  {
    counter_ = 1;
    buffer_.push_back(std::make_pair(img, camera_info));
  }
}

void CameraRecorder::setBufferDuration(float duration)
{
  boost::mutex::scoped_lock lock_bufferize( mutex_ );
  buffer_size_ = static_cast<size_t>(duration*(conv_frequency_/max_counter_));
  buffer_duration_ = duration;
  buffer_.set_capacity(buffer_size_);
}

} //publisher
} // naoqi
