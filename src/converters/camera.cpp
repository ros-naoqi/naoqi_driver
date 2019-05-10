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
#include "camera_info_definitions.hpp"
#include "../tools/alvisiondefinitions.h" // for kTop...
#include "../tools/from_any_value.hpp"

/*
* ROS includes
*/
#include <cv_bridge/cv_bridge.h>
#include <ros/console.h>

/*
* CV includes
*/
#include <opencv2/imgproc/imgproc.hpp>

/*
* BOOST includes
*/
#include <boost/foreach.hpp>
#define for_each BOOST_FOREACH

namespace naoqi
{
namespace converter
{

namespace camera_info_definitions
{

const sensor_msgs::CameraInfo& getEmptyInfo()
{
  static const sensor_msgs::CameraInfo cam_info_msg;
  return cam_info_msg;
}

const sensor_msgs::CameraInfo& getCameraInfo( int camera_source, int resolution )
{
  /** RETURN VALUE OPTIMIZATION (RVO)
  * since there is no C++11 initializer list nor lamda functions
  */
  if ( camera_source == AL::kTopCamera)
  {
    if ( resolution == AL::kVGA )
    {
      static const sensor_msgs::CameraInfo cam_info_msg = createCameraInfoTOPVGA();
      return cam_info_msg;
    }
    else if( resolution == AL::kQVGA )
    {
      static const sensor_msgs::CameraInfo cam_info_msg = createCameraInfoTOPQVGA();
      return cam_info_msg;
    }
    else if( resolution == AL::kQQVGA )
    {
      static const sensor_msgs::CameraInfo cam_info_msg = createCameraInfoTOPQQVGA();
      return cam_info_msg;
    }
  }
  else if ( camera_source == AL::kBottomCamera )
  {
    if ( resolution == AL::kVGA )
    {
      static const sensor_msgs::CameraInfo cam_info_msg = createCameraInfoBOTTOMVGA();
      return cam_info_msg;
    }
    else if( resolution == AL::kQVGA )
    {
      static const sensor_msgs::CameraInfo cam_info_msg = createCameraInfoBOTTOMQVGA();
      return cam_info_msg;
    }
    else if( resolution == AL::kQQVGA )
    {
      static const sensor_msgs::CameraInfo cam_info_msg = createCameraInfoBOTTOMQQVGA();
      return cam_info_msg;
    }
  }
  else if ( camera_source == AL::kDepthCamera )
  {
    if ( resolution == AL::kVGA )
    {
      static const sensor_msgs::CameraInfo cam_info_msg = createCameraInfoDEPTHVGA();
      ROS_WARN("VGA resolution is not supported for the depth camera, use QVGA or lower");
      return cam_info_msg;
    }
    else if( resolution == AL::kQVGA )
    {
      static const sensor_msgs::CameraInfo cam_info_msg = createCameraInfoDEPTHQVGA();
      return cam_info_msg;
    }
    else if( resolution == AL::kQQVGA )
    {
      static const sensor_msgs::CameraInfo cam_info_msg = createCameraInfoDEPTHQQVGA();
      return cam_info_msg;
    }
    else if (resolution == AL::k720p){
        static const sensor_msgs::CameraInfo cam_info_msg = createCameraInfoDEPTH720P();
        return cam_info_msg;
    }
    else if (resolution == AL::kQ720p){
        static const sensor_msgs::CameraInfo cam_info_msg = createCameraInfoDEPTHQ720P();
        return cam_info_msg;
    }
    else if (resolution == AL::kQQ720p){
        static const sensor_msgs::CameraInfo cam_info_msg = createCameraInfoDEPTHQQ720P();
        return cam_info_msg;
    }
    else if (resolution == AL::kQQQ720p){
        static const sensor_msgs::CameraInfo cam_info_msg = createCameraInfoDEPTHQQQ720P();
        return cam_info_msg;
    }
    else if (resolution == AL::kQQQQ720p){
        static const sensor_msgs::CameraInfo cam_info_msg = createCameraInfoDEPTHQQQQ720P();
        return cam_info_msg;
    }
  }
  else if (camera_source == AL::kInfraredOrStereoCamera) {
    if ( resolution == AL::kVGA )
    {
      static const sensor_msgs::CameraInfo cam_info_msg = createCameraInfoDEPTHVGA();
      ROS_WARN("VGA resolution is not supported for the depth camera, use QVGA or lower");
      return cam_info_msg;
    }
    else if( resolution == AL::kQVGA )
    {
      static const sensor_msgs::CameraInfo cam_info_msg = createCameraInfoDEPTHQVGA();
      return cam_info_msg;
    }
    else if( resolution == AL::kQQVGA )
    {
      static const sensor_msgs::CameraInfo cam_info_msg = createCameraInfoDEPTHQQVGA();
      return cam_info_msg;
    }
    else if (resolution == AL::k720px2){
      static const sensor_msgs::CameraInfo cam_info_msg = createCameraInfoStereo720PX2();
      return cam_info_msg;
    }
    else if (resolution == AL::kQ720px2){
      static const sensor_msgs::CameraInfo cam_info_msg = createCameraInfoStereoQ720PX2();
      return cam_info_msg;
    }
    else if (resolution == AL::kQQ720px2){
      static const sensor_msgs::CameraInfo cam_info_msg = createCameraInfoStereoQQ720PX2();
      return cam_info_msg;
    }
    else if (resolution == AL::kQQQ720px2){
      static const sensor_msgs::CameraInfo cam_info_msg = createCameraInfoStereoQQQ720PX2();
      return cam_info_msg;
    }
    else if (resolution == AL::kQQQQ720px2){
      static const sensor_msgs::CameraInfo cam_info_msg = createCameraInfoStereoQQQQ720PX2();
      return cam_info_msg;
    }
  }
  else{
    std::cout << "no camera information found for camera_source " << camera_source << " and res: " << resolution << std::endl;
    return getEmptyInfo();
  }
}

} // camera_info_definitions

CameraConverter::CameraConverter(
  const std::string& name,
  const float& frequency,
  const qi::SessionPtr& session,
  const int& camera_source,
  const int& resolution,
  const bool& has_stereo) : BaseConverter( name, frequency, session ),
    p_video_( session->service("ALVideoDevice") ),
    camera_source_(camera_source),
    resolution_(resolution),
    // change in case of depth camera
    colorspace_( (camera_source_!=AL::kDepthCamera)?AL::kRGBColorSpace:AL::kRawDepthColorSpace ),
    msg_colorspace_( (camera_source_!=AL::kDepthCamera)?"rgb8":"16UC1" ),
    cv_mat_type_( (camera_source_!=AL::kDepthCamera)?CV_8UC3:CV_16U ),
    camera_info_( camera_info_definitions::getCameraInfo(camera_source, resolution) )
{
  switch (camera_source) {
    case AL::kTopCamera:
      msg_frameid_ = "CameraTop_optical_frame";
      break;

    case AL::kBottomCamera:
      msg_frameid_ = "CameraBottom_optical_frame";
      break;

    case AL::kDepthCamera:
      msg_frameid_ = "CameraDepth_optical_frame";

      if (has_stereo)
        colorspace_ = AL::kDepthColorSpace;

      break;

    case AL::kInfraredOrStereoCamera:
      msg_frameid_ = "CameraDepth_optical_frame";

      if (!has_stereo) {
        camera_source_ = AL::kDepthCamera;
        colorspace_ = AL::kInfraredColorSpace;
        msg_colorspace_ = "16UC1";
        cv_mat_type_ = CV_16U;
      }

      camera_info_ = camera_info_definitions::getCameraInfo(camera_source_, resolution_);
      break;
  }
}

CameraConverter::~CameraConverter()
{
  if (!handle_.empty())
  {
    std::cout << "Unsubscribe camera handle " << handle_ << std::endl;
    p_video_.call<qi::AnyValue>("unsubscribe", handle_);
    handle_.clear();
  }
}

void CameraConverter::reset()
{
  if (!handle_.empty())
  {
    p_video_.call<qi::AnyValue>("unsubscribe", handle_);
    handle_.clear();
  }

  handle_ = p_video_.call<std::string>(
                         "subscribeCamera",
                          name_,
                          camera_source_,
                          resolution_,
                          colorspace_,
                          frequency_
                          );
}

void CameraConverter::registerCallback( const message_actions::MessageAction action, Callback_t cb )
{
  callbacks_[action] = cb;
}

void CameraConverter::callAll( const std::vector<message_actions::MessageAction>& actions )
{

  if (handle_.empty() )
  {
    std::cerr << name_ << "Camera Handle is empty - cannot retrieve image" << std::endl;
    std::cerr << name_ << "Might be a NAOqi problem. Try to restart the ALVideoDevice." << std::endl;
    return;
  }

  qi::AnyValue image_anyvalue = p_video_.call<qi::AnyValue>("getImageRemote", handle_);
  tools::NaoqiImage image;
  try{
      image = tools::fromAnyValueToNaoqiImage(image_anyvalue);
  }
  catch(std::runtime_error& e)
  {
    std::cout << "Cannot retrieve image" << std::endl;
    return;
  }

  // Create a cv::Mat of the right dimensions
  cv::Mat cv_img(image.height, image.width, cv_mat_type_, image.buffer);
  msg_ = cv_bridge::CvImage(std_msgs::Header(), msg_colorspace_, cv_img).toImageMsg();
  msg_->header.frame_id = msg_frameid_;

  msg_->header.stamp = ros::Time::now();
  //msg_->header.stamp.sec = image.timestamp_s;
  //msg_->header.stamp.nsec = image.timestamp_us*1000;
  camera_info_.header.stamp = msg_->header.stamp;

  for_each( const message_actions::MessageAction& action, actions )
  {
    callbacks_[action]( msg_, camera_info_ );
  }
}

} // publisher
} //naoqi
