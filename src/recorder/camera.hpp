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

#ifndef CAMERA_RECORDER_HPP
#define CAMERA_RECORDER_HPP

/*
* LOCAL includes
*/
#include <alrosbridge/recorder/globalrecorder.hpp>
#include "../helpers.hpp"

/*
* ROS includes
*/
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

namespace alros
{
namespace recorder
{

class CameraRecorder
{

public:
  CameraRecorder(const std::string& topic, float buffer_frequency );

  void write( const sensor_msgs::ImagePtr& img, const sensor_msgs::CameraInfo& camera_info );

  void reset(boost::shared_ptr<alros::recorder::GlobalRecorder> gr, float conv_frequency );

  void bufferize( const sensor_msgs::ImagePtr& img, const sensor_msgs::CameraInfo& camera_info );

  void writeDump();

  void setBufferDuration(float duration);

  inline std::string topic() const
  {
    return topic_img_;
  }

  inline bool isInitialized() const
  {
    return is_initialized_;
  }

  inline void subscribe( bool state)
  {
    is_subscribed_ = state;
  }

  inline bool isSubscribed() const
  {
    return is_subscribed_;
  }

protected:
  bool is_initialized_;
  bool is_subscribed_;

  std::list< std::pair<sensor_msgs::ImagePtr, sensor_msgs::CameraInfo> > buffer_;
  size_t buffer_size_;
  float buffer_duration_;

  boost::mutex mutex_;

  boost::shared_ptr<alros::recorder::GlobalRecorder> gr_;
  std::string topic_info_;
  std::string topic_img_;

  float buffer_frequency_;
  float conv_frequency_;
  int counter_;
  int max_counter_;

}; // class

} //publisher
} // alros

#endif
