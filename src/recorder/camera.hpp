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

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include "recorderbase.hpp"
#include <alrosbridge/recorder/globalrecorder.hpp>
#include <qi/anyobject.hpp>

namespace alros
{
namespace recorder
{

class CameraRecorder : public BaseRecorder<CameraRecorder>
{

public:
  CameraRecorder( const std::string& topic );

  void write( const sensor_msgs::ImagePtr& img, const sensor_msgs::CameraInfo& camera_info );

  void reset( boost::shared_ptr<alros::recorder::GlobalRecorder> gr );

private:
  boost::shared_ptr<alros::recorder::GlobalRecorder> gr_;
  std::string topic_info_;
  std::string topic_img_;

}; // class

} //publisher
} // alros

#endif
