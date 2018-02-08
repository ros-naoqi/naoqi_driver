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

#include "touch.hpp"

namespace naoqi
{

template<class T>
TouchEventRegister<T>::TouchEventRegister()
{
}

template<class T>
TouchEventRegister<T>::TouchEventRegister( const std::string& name, const std::vector<std::string> keys, const float& frequency, const qi::SessionPtr& session )
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
  converter_ = boost::make_shared<converter::TouchEventConverter<T> >( name, frequency, session );

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
TouchEventRegister<T>::~TouchEventRegister()
{
  stopProcess();
}

template<class T>
void TouchEventRegister<T>::resetPublisher(ros::NodeHandle& nh)
{
  publisher_->reset(nh);
}

template<class T>
void TouchEventRegister<T>::resetRecorder( boost::shared_ptr<naoqi::recorder::GlobalRecorder> gr )
{
  //recorder_->reset(gr, converter_->frequency());
}

template<class T>
void TouchEventRegister<T>::startProcess()
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
        p_memory_.call<void>("subscribeToEvent",it->c_str(), serviceName, "touchCallback");
      }
      std::cout << serviceName << " : Start" << std::endl;
    }
    isStarted_ = true;
  }
}

template<class T>
void TouchEventRegister<T>::stopProcess()
{
  boost::mutex::scoped_lock stop_lock(mutex_);
  if (isStarted_)
  {
    //std::string serviceName = std::string("ROS-Driver-") + typeid(T).name();
    std::string serviceName = std::string("ROS-Driver-") + keys_[0];
    if(serviceId){
      for(std::vector<std::string>::const_iterator it = keys_.begin(); it != keys_.end(); ++it) {
        p_memory_.call<void>("unsubscribeToEvent",it->c_str(), serviceName);
      }
      session_->unregisterService(serviceId);
      serviceId = 0;
    }
    std::cout << serviceName << " : Stop" << std::endl;
    isStarted_ = false;
  }
}

template<class T>
void TouchEventRegister<T>::writeDump(const ros::Time& time)
{
  if (isStarted_)
  {
    //recorder_->writeDump(time);
  }
}

template<class T>
void TouchEventRegister<T>::setBufferDuration(float duration)
{
  //recorder_->setBufferDuration(duration);
}

template<class T>
void TouchEventRegister<T>::isRecording(bool state)
{
  boost::mutex::scoped_lock rec_lock(mutex_);
  isRecording_ = state;
}

template<class T>
void TouchEventRegister<T>::isPublishing(bool state)
{
  boost::mutex::scoped_lock pub_lock(mutex_);
  isPublishing_ = state;
}

template<class T>
void TouchEventRegister<T>::isDumping(bool state)
{
  boost::mutex::scoped_lock dump_lock(mutex_);
  isDumping_ = state;
}

template<class T>
void TouchEventRegister<T>::registerCallback()
{
}

template<class T>
void TouchEventRegister<T>::unregisterCallback()
{
}

template<class T>
void TouchEventRegister<T>::touchCallback(std::string &key, qi::AnyValue &value, qi::AnyValue &message)
{
  T msg = T();
  
  bool state =  value.toFloat() > 0.5f;

  //std::cerr << key << " " << state << std::endl;

  touchCallbackMessage(key, state, msg);

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
void TouchEventRegister<T>::touchCallbackMessage(std::string &key, bool &state, naoqi_bridge_msgs::Bumper &msg)
{
  int i = 0;
  for(std::vector<std::string>::const_iterator it = keys_.begin(); it != keys_.end(); ++it, ++i)
  {
    if ( key == it->c_str() ) {
      msg.bumper = i;
      msg.state = state?(naoqi_bridge_msgs::Bumper::statePressed):(naoqi_bridge_msgs::Bumper::stateReleased);
    }
  }
}

template<class T>
void TouchEventRegister<T>::touchCallbackMessage(std::string &key, bool &state, naoqi_bridge_msgs::HandTouch &msg)
{
  int i = 0;
  for(std::vector<std::string>::const_iterator it = keys_.begin(); it != keys_.end(); ++it, ++i)
  {
    if ( key == it->c_str() ) {
      msg.hand = i;
      msg.state = state?(naoqi_bridge_msgs::HandTouch::statePressed):(naoqi_bridge_msgs::HandTouch::stateReleased);
    }
  }
}

template<class T>
void TouchEventRegister<T>::touchCallbackMessage(std::string &key, bool &state, naoqi_bridge_msgs::HeadTouch &msg)
{
  int i = 0;
  for(std::vector<std::string>::const_iterator it = keys_.begin(); it != keys_.end(); ++it, ++i)
  {
    if ( key == it->c_str() ) {
      msg.button = i;
      msg.state = state?(naoqi_bridge_msgs::HeadTouch::statePressed):(naoqi_bridge_msgs::HeadTouch::stateReleased);
    }
  }
}

template<class T>
void TouchEventRegister<T>::touchCallbackMessage(std::string &key, bool &state, naoqi_bridge_msgs::ChestButtonPressed &msg)
{
  for(std::vector<std::string>::const_iterator it = keys_.begin(); it != keys_.end(); ++it)
  {
    if ( key == it->c_str() ) {
      msg.state = state?(naoqi_bridge_msgs::ChestButtonPressed::statePressed):(naoqi_bridge_msgs::ChestButtonPressed::stateReleased);
    }
  }
}

// http://stackoverflow.com/questions/8752837/undefined-reference-to-template-class-constructor
template class TouchEventRegister<naoqi_bridge_msgs::Bumper>;
template class TouchEventRegister<naoqi_bridge_msgs::HandTouch>;
template class TouchEventRegister<naoqi_bridge_msgs::HeadTouch>;
template class TouchEventRegister<naoqi_bridge_msgs::ChestButtonPressed>;

}//namespace
