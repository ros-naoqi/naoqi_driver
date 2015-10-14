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
#include "photocapture.hpp"


namespace naoqi
{
namespace subscriber
{

  PhotoCaptureSubscriber::PhotoCaptureSubscriber( const std::string& name, const std::string& photocapture_topic, const qi::SessionPtr& session):
    photocapture_topic_(photocapture_topic),
    BaseSubscriber( name, photocapture_topic, session ),
    p_photocapture_( session->service("ALPhotoCapture") )
  {}

  void PhotoCaptureSubscriber::reset( ros::NodeHandle& nh )
  {
    sub_photocapture_ = nh.subscribe( photocapture_topic_, 10, &PhotoCaptureSubscriber::callback, this );
    is_initialized_ = true;
  }
  void PhotoCaptureSubscriber::callback( const std_msgs::StringConstPtr& string_msg )
  {
    p_photocapture_.async<void>("setResolution", 2);
    p_photocapture_.async<void>("setPictureFormat", "jpg");
    p_photocapture_.async<void>("takePicture", "/home/nao/recordings/cameras/", string_msg->data);
    std::cout << "photo name: "<<string_msg->data <<".jpg saved in /home/nao/recordings/cameras/"<< std::endl;  
  }

} //subscriber
} // naoqi
