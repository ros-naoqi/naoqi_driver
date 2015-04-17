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
#include "camera.hpp"

namespace alros
{
namespace recorder
{

CameraRecorder::CameraRecorder( const std::string& topic_ )
{
  topic_info_ = topic_ + "/camera_info";
  topic_img_ = topic_ + "/image";
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

void CameraRecorder::reset(boost::shared_ptr<GlobalRecorder> gr)
{
  gr_ = gr;
  is_initialized_ = true;
}

} //publisher
} // alros
