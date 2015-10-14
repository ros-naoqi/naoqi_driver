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

BumperEventRegister::BumperEventRegister()
{
}

BumperEventRegister::BumperEventRegister( const std::string& name, const std::vector<std::string> keys, const float& frequency, const qi::SessionPtr& session )
  : serviceId(0),
    p_memory_( session->service("ALMemory")),
    p_touch_( session->service("ALTouch")),
    session_(session),
    isStarted_(false),
    isPublishing_(false),
    isRecording_(false),
    isDumping_(false)
{
  publisher_ = boost::make_shared<publisher::BasicPublisher<naoqi_bridge_msgs::Bumper> >( name );
  //recorder_ = boost::make_shared<recorder::BasicEventRecorder<naoqi_bridge_msgs::Bumper> >( name );
  converter_ = boost::make_shared<converter::BumperEventConverter>( name, frequency, session );

  converter_->registerCallback( message_actions::PUBLISH, boost::bind(&publisher::BasicPublisher<naoqi_bridge_msgs::Bumper>::publish, publisher_, _1) );
  //converter_->registerCallback( message_actions::RECORD, boost::bind(&recorder::BasicEventRecorder<naoqi_bridge_msgs::Bumper>::write, recorder_, _1) );
  //converter_->registerCallback( message_actions::LOG, boost::bind(&recorder::BasicEventRecorder<naoqi_bridge_msgs::Bumper>::bufferize, recorder_, _1) );

  keys_.resize(keys.size());
  size_t i = 0;
  for(std::vector<std::string>::const_iterator it = keys.begin(); it != keys.end(); ++it, ++i)
    keys_[i] = *it;

  name_ = name;
}

BumperEventRegister::~BumperEventRegister()
{
  stopProcess();
}

void BumperEventRegister::resetPublisher(ros::NodeHandle& nh)
{
  publisher_->reset(nh);
}

void BumperEventRegister::resetRecorder( boost::shared_ptr<naoqi::recorder::GlobalRecorder> gr )
{
  //recorder_->reset(gr, converter_->frequency());
}

void BumperEventRegister::startProcess()
{
  boost::mutex::scoped_lock start_lock(mutex_);
  if (!isStarted_)
  {
    if(!serviceId)
    {
      serviceId = session_->registerService("ROS-Driver-Bumper", shared_from_this());
      for(std::vector<std::string>::const_iterator it = keys_.begin(); it != keys_.end(); ++it)
        p_memory_.call<void>("subscribeToEvent",it->c_str(), "ROS-Driver-Bumper", "touchCallback");
      std::cout << "Bumper : Start" << std::endl;
    }
    isStarted_ = true;
  }
}

void BumperEventRegister::stopProcess()
{
  boost::mutex::scoped_lock stop_lock(mutex_);
  if (isStarted_)
  {
    if(serviceId){
      p_touch_.call<void>("unsubscribeToEvent", "ROS-Driver-Bumper");
      session_->unregisterService(serviceId);
      serviceId = 0;
    }
    std::cout << "Bumper: Stop" << std::endl;
    isStarted_ = false;
  }
}

void BumperEventRegister::writeDump(const ros::Time& time)
{
  if (isStarted_)
  {
    //recorder_->writeDump(time);
  }
}

void BumperEventRegister::setBufferDuration(float duration)
{
  //recorder_->setBufferDuration(duration);
}

void BumperEventRegister::isRecording(bool state)
{
  boost::mutex::scoped_lock rec_lock(mutex_);
  isRecording_ = state;
}

void BumperEventRegister::isPublishing(bool state)
{
  boost::mutex::scoped_lock pub_lock(mutex_);
  isPublishing_ = state;
}

void BumperEventRegister::isDumping(bool state)
{
  boost::mutex::scoped_lock dump_lock(mutex_);
  isDumping_ = state;
}

void BumperEventRegister::registerCallback()
{
}

void BumperEventRegister::unregisterCallback()
{
}

void BumperEventRegister::touchCallback(std::string &key, qi::AnyValue &value, qi::AnyValue &message)
{
  naoqi_bridge_msgs::Bumper msg = naoqi_bridge_msgs::Bumper();
  
  bool state =  value.toFloat() > 0.5f;

  std::cerr << key << " " << state << std::endl;
  int i = 0;
  for(std::vector<std::string>::const_iterator it = keys_.begin(); it != keys_.end(); ++it, ++i)
  {
    if ( key == it->c_str() ) {
      msg.bumper = i;
      msg.state = state?(naoqi_bridge_msgs::Bumper::statePressed):(naoqi_bridge_msgs::Bumper::stateReleased);
    }
  }    

  isPublishing_ = true;
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

////

TactileTouchEventRegister::TactileTouchEventRegister()
{
}

TactileTouchEventRegister::TactileTouchEventRegister( const std::string& name, const std::vector<std::string> keys, const float& frequency, const qi::SessionPtr& session )
  : serviceId(0),
    p_memory_( session->service("ALMemory")),
    p_touch_( session->service("ALTouch")),
    session_(session),
    isStarted_(false),
    isPublishing_(false),
    isRecording_(false),
    isDumping_(false)
{
  publisher_ = boost::make_shared<publisher::BasicPublisher<naoqi_bridge_msgs::TactileTouch> >( name );
  //recorder_ = boost::make_shared<recorder::BasicEventRecorder<naoqi_bridge_msgs::Bumper> >( name );
  converter_ = boost::make_shared<converter::TactileTouchEventConverter>( name, frequency, session );

  converter_->registerCallback( message_actions::PUBLISH, boost::bind(&publisher::BasicPublisher<naoqi_bridge_msgs::TactileTouch>::publish, publisher_, _1) );
  //converter_->registerCallback( message_actions::RECORD, boost::bind(&recorder::BasicEventRecorder<naoqi_bridge_msgs::Bumper>::write, recorder_, _1) );
  //converter_->registerCallback( message_actions::LOG, boost::bind(&recorder::BasicEventRecorder<naoqi_bridge_msgs::Bumper>::bufferize, recorder_, _1) );

  keys_.resize(keys.size());
  size_t i = 0;
  for(std::vector<std::string>::const_iterator it = keys.begin(); it != keys.end(); ++it, ++i)
    keys_[i] = *it;

  name_ = name;
}

TactileTouchEventRegister::~TactileTouchEventRegister()
{
  stopProcess();
}

void TactileTouchEventRegister::resetPublisher(ros::NodeHandle& nh)
{
  publisher_->reset(nh);
}

void TactileTouchEventRegister::resetRecorder( boost::shared_ptr<naoqi::recorder::GlobalRecorder> gr )
{
  //recorder_->reset(gr, converter_->frequency());
}

void TactileTouchEventRegister::startProcess()
{
  boost::mutex::scoped_lock start_lock(mutex_);
  if (!isStarted_)
  {
    if(!serviceId)
    {
      serviceId = session_->registerService("ROS-Driver-TactileTouch", shared_from_this());
      for(std::vector<std::string>::const_iterator it = keys_.begin(); it != keys_.end(); ++it)
        p_memory_.call<void>("subscribeToEvent",it->c_str(), "ROS-Driver-TactileTouch", "touchCallback");
      std::cout << "TactileTouch : Start" << std::endl;
    }
    isStarted_ = true;
  }
}

void TactileTouchEventRegister::stopProcess()
{
  boost::mutex::scoped_lock stop_lock(mutex_);
  if (isStarted_)
  {
    if(serviceId){
      p_touch_.call<void>("unsubscribeToEvent", "ROS-Driver-TactileTouch");
      session_->unregisterService(serviceId);
      serviceId = 0;
    }
    std::cout << "TactileTouch: Stop" << std::endl;
    isStarted_ = false;
  }
}

void TactileTouchEventRegister::writeDump(const ros::Time& time)
{
  if (isStarted_)
  {
    //recorder_->writeDump(time);
  }
}

void TactileTouchEventRegister::setBufferDuration(float duration)
{
  //recorder_->setBufferDuration(duration);
}

void TactileTouchEventRegister::isRecording(bool state)
{
  boost::mutex::scoped_lock rec_lock(mutex_);
  isRecording_ = state;
}

void TactileTouchEventRegister::isPublishing(bool state)
{
  boost::mutex::scoped_lock pub_lock(mutex_);
  isPublishing_ = state;
}

void TactileTouchEventRegister::isDumping(bool state)
{
  boost::mutex::scoped_lock dump_lock(mutex_);
  isDumping_ = state;
}

void TactileTouchEventRegister::registerCallback()
{
}

void TactileTouchEventRegister::unregisterCallback()
{
}

void TactileTouchEventRegister::touchCallback(std::string &key, qi::AnyValue &value, qi::AnyValue &message)
{
  naoqi_bridge_msgs::TactileTouch msg = naoqi_bridge_msgs::TactileTouch();
  
  bool state =  value.toFloat() > 0.5f;

  std::cerr << key << " " << state << std::endl;
  int i = 0;
  for(std::vector<std::string>::const_iterator it = keys_.begin(); it != keys_.end(); ++it, ++i)
  {
    if ( key == it->c_str() ) {
      msg.button = i;
      msg.state = state?(naoqi_bridge_msgs::TactileTouch::statePressed):(naoqi_bridge_msgs::TactileTouch::stateReleased);
    }
  }    

  isPublishing_ = true;
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

QI_REGISTER_OBJECT(BumperEventRegister, touchCallback)
QI_REGISTER_OBJECT(TactileTouchEventRegister, touchCallback)

}//namespace
