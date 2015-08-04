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

#ifndef CONVERTER_CAMERA_HPP
#define CONVERTER_CAMERA_HPP

/*
* LOCAL includes
*/
#include "converter_base.hpp"
#include <naoqi_driver/message_actions.h>

/*
* ROS includes
*/
#include <image_transport/image_transport.h>

namespace naoqi
{
namespace converter
{

class CameraConverter : public BaseConverter<CameraConverter>
{

  typedef boost::function<void(sensor_msgs::ImagePtr, sensor_msgs::CameraInfo)> Callback_t;

public:
  CameraConverter( const std::string& name, const float& frequency, const qi::SessionPtr& session, const int& camera_source, const int& resolution );

  ~CameraConverter();

  void reset();

  void registerCallback( const message_actions::MessageAction action, Callback_t cb );

  void callAll( const std::vector<message_actions::MessageAction>& actions );

private:
  std::map<message_actions::MessageAction, Callback_t> callbacks_;

  /** VideoDevice (Proxy) configurations */
  qi::AnyObject p_video_;
  int camera_source_;
  int resolution_;
  int colorspace_;
  std::string handle_;

  // string indicating image transport encoding
  // goes along with colorspace_
  std::string msg_colorspace_;
  int cv_mat_type_;
  // msg frame id
  std::string msg_frameid_;
  sensor_msgs::CameraInfo camera_info_;
  sensor_msgs::ImagePtr msg_;
};

} //publisher
} //naoqi


#endif
