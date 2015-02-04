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

#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <alvision/alvisiondefinitions.h> // for kTop...
#include <alvision/alimage.h>
#include <alvalue/alvalue.h>
#include <alvision/alimage_opencv.h>


#include "camera.hpp"

namespace alros
{
namespace publisher
{

CameraPublisher::CameraPublisher( const std::string& name, const std::string& topic, float frequency, const qi::AnyObject& p_video )
  : BasePublisher( name, topic, frequency ),
    p_video_( p_video )
{}

void CameraPublisher::publish()
{

  // THIS WILL CRASH IN THE FUTURE
  AL::ALValue value = p_video_.call<AL::ALValue>("getImageRemote", handle_);

  AL::ALImage* img_buffer = AL::ALImage::fromALValue(value); // allocate memory -> call delete later
  cv::Mat cv_img = AL::aLImageToCvMat( *img_buffer );
  msg_ = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_img).toImageMsg();
  delete img_buffer;

  pub_.publish( msg_ );
}

void CameraPublisher::reset( ros::NodeHandle& nh )
{

  // check for double subscribers
  // if handle_ is not null i continue or I unsubscribe and re-subscribe again
  // if handle_ is null, i subscribe

  // ALSO WICHTIG:: unsubscribe in desctructor
  // check for re-init of video device
  if ( handle_.empty() )
  {
    handle_ = p_video_.call<std::string>(
                         "subscribeCamera",
                          name_,
                          AL::kTopCamera,
                          AL::getResolutionFromSize(160, 120),
                          AL::kBGRColorSpace,
                          20
                          );
  }
  image_transport::ImageTransport it( nh );
  pub_ = it.advertise( topic_, 1 );

  is_initialized_ = true;

  std::cout << "image device is totally initialized" << std::endl;

}

} // publisher
} //alros
