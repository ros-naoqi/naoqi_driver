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

#include <iostream>
#include <vector>

#include <boost/make_shared.hpp>

#include <ros/ros.h>

#include <qi/anyobject.hpp>

#include <naoqi_driver/recorder/globalrecorder.hpp>
#include <naoqi_driver/message_actions.h>
#include "../tools/from_any_value.hpp"

#include "people.hpp"

namespace naoqi
{

template<class T>
PeopleEventRegister<T>::PeopleEventRegister()
{
}

template<class T>
PeopleEventRegister<T>::PeopleEventRegister( const std::string& name, const std::vector<std::string> keys, const float& frequency, const qi::SessionPtr& session )
  : serviceId(0),
    p_memory_( session->service("ALMemory")),
    session_(session),
    isStarted_(false),
    isPublishing_(false),
    isRecording_(false),
    isDumping_(false)
{
  publisher_ = boost::make_shared<publisher::BasicPublisher<T> >( name );
  //recorder_ = boost::make_shared<recorder::BasicEventRecorder<T> >( name );
  converter_ = boost::make_shared<converter::PeopleEventConverter<T> >( name, frequency, session );

  converter_->registerCallback( message_actions::PUBLISH, boost::bind(&publisher::BasicPublisher<T>::publish, publisher_, _1) );
  //converter_->registerCallback( message_actions::RECORD, boost::bind(&recorder::BasicEventRecorder<T>::write, recorder_, _1) );
  //converter_->registerCallback( message_actions::LOG, boost::bind(&recorder::BasicEventRecorder<T>::bufferize, recorder_, _1) );

  keys_.resize(keys.size());
  size_t i = 0;
  for(std::vector<std::string>::const_iterator it = keys.begin(); it != keys.end(); ++it, ++i)
    keys_[i] = *it;

  name_ = name;
}

template<class T>
PeopleEventRegister<T>::~PeopleEventRegister()
{
  stopProcess();
}

template<class T>
void PeopleEventRegister<T>::resetPublisher(ros::NodeHandle& nh)
{
  publisher_->reset(nh);
}

template<class T>
void PeopleEventRegister<T>::resetRecorder( boost::shared_ptr<naoqi::recorder::GlobalRecorder> gr )
{
  //recorder_->reset(gr, converter_->frequency());
}

template<class T>
void PeopleEventRegister<T>::startProcess()
{
  boost::mutex::scoped_lock start_lock(mutex_);
  if (!isStarted_)
  {
    if(!serviceId)
    {
      //std::string serviceName = std::string("ROS-Driver-") + typeid(T).name();
      std::string serviceName = std::string("ROS-Driver-") + keys_[0];
      serviceId = session_->registerService(serviceName, this->shared_from_this());
      for(std::vector<std::string>::const_iterator it = keys_.begin(); it != keys_.end(); ++it) {
        std::cerr << *it << std::endl;
        p_memory_.call<void>("subscribeToEvent",it->c_str(), serviceName, "peopleCallback");
      }
      std::cout << serviceName << " : Start" << std::endl;
    }
    isStarted_ = true;
  }
}

template<class T>
void PeopleEventRegister<T>::stopProcess()
{
  boost::mutex::scoped_lock stop_lock(mutex_);
  if (isStarted_)
  {
    //std::string serviceName = std::string("ROS-Driver-") + typeid(T).name();
    std::string serviceName = std::string("ROS-Driver-") + keys_[0];
    if(serviceId){
      p_memory_.call<void>("unsubscribeToEvent", serviceName, "peopleCallback");
      session_->unregisterService(serviceId);
      serviceId = 0;
    }
    std::cout << serviceName << " : Stop" << std::endl;
    isStarted_ = false;
  }
}

template<class T>
void PeopleEventRegister<T>::writeDump(const ros::Time& time)
{
  if (isStarted_)
  {
    //recorder_->writeDump(time);
  }
}

template<class T>
void PeopleEventRegister<T>::setBufferDuration(float duration)
{
  //recorder_->setBufferDuration(duration);
}

template<class T>
void PeopleEventRegister<T>::isRecording(bool state)
{
  boost::mutex::scoped_lock rec_lock(mutex_);
  isRecording_ = state;
}

template<class T>
void PeopleEventRegister<T>::isPublishing(bool state)
{
  boost::mutex::scoped_lock pub_lock(mutex_);
  isPublishing_ = state;
}

template<class T>
void PeopleEventRegister<T>::isDumping(bool state)
{
  boost::mutex::scoped_lock dump_lock(mutex_);
  isDumping_ = state;
}

template<class T>
void PeopleEventRegister<T>::registerCallback()
{
}

template<class T>
void PeopleEventRegister<T>::unregisterCallback()
{
}

template<class T>
void PeopleEventRegister<T>::peopleCallback(std::string &key, qi::AnyValue &value, qi::AnyValue &message)
{
  T msg = T();
  
  tools::NaoqiFaceDetected faces;
  try {
    faces = tools::fromAnyValueToNaoqiFaceDetected(value);
  }
  catch(std::runtime_error& e)
  {
    std::cout << "Cannot retrieve facedetect" << std::endl;
    return;
  }

  if ( faces.face_info.size() > 0 ) { // sometimes value does not have face information..
    peopleCallbackMessage(key, faces, msg);
  }

  std::vector<message_actions::MessageAction> actions;
  boost::mutex::scoped_lock callback_lock(mutex_);

  if (isStarted_) {
    // CHECK FOR PUBLISH
    if ( isPublishing_ && publisher_->isSubscribed() )
    {
      actions.push_back(message_actions::PUBLISH);
    }
    // CHECK FOR RECORD
    if ( isRecording_ )
    {
      //actions.push_back(message_actions::RECORD);
    }
    if ( !isDumping_ )
    {
      //actions.push_back(message_actions::LOG);
    }
    if (actions.size() >0)
    {
      converter_->callAll( actions, msg );
    }
  }
}

template<class T>
void PeopleEventRegister<T>::peopleCallbackMessage(std::string &key, tools::NaoqiFaceDetected &faces, nao_interaction_msgs::FaceDetected &msg)
{
  msg.header.frame_id = "";
  msg.header.stamp = ros::Time(faces.timestamp.timestamp_s, faces.timestamp.timestamp_us);

  if ( faces.face_info.size() > 0 ) {
    msg.face_id.data = faces.face_info[0].extra_info[0].face_id;
    msg.score_reco.data = faces.face_info[0].extra_info[0].score_reco;
    msg.face_label.data = faces.face_info[0].extra_info[0].face_label;

    msg.shape_alpha.data = faces.face_info[0].shape_info.alpha;
    msg.shape_beta.data  = faces.face_info[0].shape_info.beta;
    msg.shape_sizeX.data = faces.face_info[0].shape_info.sizeX;
    msg.shape_sizeY.data = faces.face_info[0].shape_info.sizeY;

    msg.right_eye_eyeCenter_x.data = faces.face_info[0].extra_info[0].right_eye_points.eye_center_x;
    msg.right_eye_eyeCenter_y.data = faces.face_info[0].extra_info[0].right_eye_points.eye_center_y;
    msg.right_eye_noseSideLimit_x.data = faces.face_info[0].extra_info[0].right_eye_points.nose_side_limit_x;
    msg.right_eye_noseSideLimit_y.data = faces.face_info[0].extra_info[0].right_eye_points.nose_side_limit_y;
    msg.right_eye_earSideLimit_x.data = faces.face_info[0].extra_info[0].right_eye_points.ear_side_limit_x;
    msg.right_eye_earSideLimit_y.data = faces.face_info[0].extra_info[0].right_eye_points.ear_side_limit_y;

    msg.left_eye_eyeCenter_x.data = faces.face_info[0].extra_info[0].left_eye_points.eye_center_x;
    msg.left_eye_eyeCenter_y.data = faces.face_info[0].extra_info[0].left_eye_points.eye_center_y;
    msg.left_eye_noseSideLimit_x.data = faces.face_info[0].extra_info[0].left_eye_points.nose_side_limit_x;
    msg.left_eye_noseSideLimit_y.data = faces.face_info[0].extra_info[0].left_eye_points.nose_side_limit_y;
    msg.left_eye_earSideLimit_x.data = faces.face_info[0].extra_info[0].left_eye_points.ear_side_limit_x;
    msg.left_eye_earSideLimit_y.data = faces.face_info[0].extra_info[0].left_eye_points.ear_side_limit_y;

    msg.nose_bottomCenterLimit_x.data = faces.face_info[0].extra_info[0].nose_points.bottom_center_limit_x;
    msg.nose_bottomCenterLimit_y.data = faces.face_info[0].extra_info[0].nose_points.bottom_center_limit_y;
    msg.nose_bottomLeftLimit_x.data = faces.face_info[0].extra_info[0].nose_points.bottom_left_limit_x;
    msg.nose_bottomLeftLimit_y.data = faces.face_info[0].extra_info[0].nose_points.bottom_left_limit_y;
    msg.nose_bottomRightLimit_x.data = faces.face_info[0].extra_info[0].nose_points.bottom_right_limit_x;
    msg.nose_bottomRightLimit_y.data = faces.face_info[0].extra_info[0].nose_points.bottom_right_limit_y;

    msg.mouth_leftLimit_x.data = faces.face_info[0].extra_info[0].mouth_points.left_limit_x;
    msg.mouth_leftLimit_y.data = faces.face_info[0].extra_info[0].mouth_points.left_limit_y;
    msg.mouth_rightLimit_x.data = faces.face_info[0].extra_info[0].mouth_points.right_limit_x;
    msg.mouth_rightLimit_y.data = faces.face_info[0].extra_info[0].mouth_points.right_limit_y;
    msg.mouth_topLimit_x.data = faces.face_info[0].extra_info[0].mouth_points.top_limit_x;
    msg.mouth_topLimit_y.data = faces.face_info[0].extra_info[0].mouth_points.top_limit_y;
  }
}

// http://stackoverflow.com/questions/8752837/undefined-reference-to-template-class-constructor
template class PeopleEventRegister<nao_interaction_msgs::FaceDetected>;

}//namespace
