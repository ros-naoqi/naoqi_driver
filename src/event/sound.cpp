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

#include "sound.hpp"
#include "../tools/from_any_value.hpp"

namespace naoqi
{

template<class T>
SoundEventRegister<T>::SoundEventRegister()
{
}

template<class T>
SoundEventRegister<T>::SoundEventRegister(const std::string& name, std::vector<std::string> keys, const float& frequency, const qi::SessionPtr& session )
  : serviceId(0),
    keys_(keys),
    p_memory_(session->service("ALMemory")),
    p_sound_loc_(session->service("ALSoundLocalization")),
    session_(session),
    isStarted_(false),
    isPublishing_(false),
    isRecording_(false),
    isDumping_(false)
{

  publisher_ = boost::make_shared<publisher::BasicPublisher<T> >( name );
  recorder_ = boost::make_shared<recorder::BasicEventRecorder<T> >( name );
  converter_ = boost::make_shared<converter::SoundEventConverter<T> >( name, frequency, session );

  converter_->registerCallback( message_actions::PUBLISH, boost::bind(&publisher::BasicPublisher<T>::publish, publisher_, _1) );
  converter_->registerCallback( message_actions::RECORD, boost::bind(&recorder::BasicEventRecorder<T>::write, recorder_, _1) );
  converter_->registerCallback( message_actions::LOG, boost::bind(&recorder::BasicEventRecorder<T>::bufferize, recorder_, _1) );

}

template<class T>
SoundEventRegister<T>::~SoundEventRegister()
{
  stopProcess();
}

template<class T>
void SoundEventRegister<T>::resetPublisher(ros::NodeHandle& nh)
{
  publisher_->reset(nh);
}

template<class T>
void SoundEventRegister<T>::resetRecorder( boost::shared_ptr<naoqi::recorder::GlobalRecorder> gr )
{
  recorder_->reset(gr, converter_->frequency());
}

template<class T>
void SoundEventRegister<T>::setEnergyComputation(bool run) {
  p_sound_loc_.call<void>("setParameter", "EnergyComputation", run);
}

template<class T>
void SoundEventRegister<T>::setSensitivity(float level) {
  p_sound_loc_.call<void>("setParameter", "Sensitivity", level);
}

template<class T>
void SoundEventRegister<T>::startProcess()
{
  boost::mutex::scoped_lock start_lock(mutex_);
  if (!isStarted_)
  {
    if(!serviceId)
    {
      std::string serviceName = std::string("ROS-Driver-") + keys_[0];
      serviceId = session_->registerService(serviceName, this->shared_from_this());
      for(std::vector<std::string>::const_iterator it = keys_.begin(); it != keys_.end(); ++it) {
        std::cerr << *it << std::endl;
        p_memory_.call<void>("subscribeToEvent",it->c_str(), serviceName, "soundCallback");
      }
      std::cout << serviceName << " : Start" << std::endl;
    }
    isStarted_ = true;
  }
}

template<class T>
void SoundEventRegister<T>::stopProcess()
{
  boost::mutex::scoped_lock stop_lock(mutex_);
  if (isStarted_)
  {
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
void SoundEventRegister<T>::writeDump(const ros::Time& time)
{
  if (isStarted_)
  {
    recorder_->writeDump(time);
  }
}

template<class T>
void SoundEventRegister<T>::setBufferDuration(float duration)
{
  recorder_->setBufferDuration(duration);
}

template<class T>
void SoundEventRegister<T>::isRecording(bool state)
{
  boost::mutex::scoped_lock rec_lock(mutex_);
  isRecording_ = state;
}

template<class T>
void SoundEventRegister<T>::isPublishing(bool state)
{
  boost::mutex::scoped_lock pub_lock(mutex_);
  isPublishing_ = state;
}

template<class T>
void SoundEventRegister<T>::isDumping(bool state)
{
  boost::mutex::scoped_lock dump_lock(mutex_);
  isDumping_ = state;
}

template<class T>
void SoundEventRegister<T>::registerCallback()
{
}

template<class T>
void SoundEventRegister<T>::unregisterCallback()
{
}

template<class T>
void SoundEventRegister<T>::soundCallback(std::string &key, qi::AnyValue &value, qi::AnyValue &message) {
    T msg = T();
  
    soundCallbackMessage(key, value, msg);
    
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
        actions.push_back(message_actions::RECORD);
      }
      if ( !isDumping_ )
      {
        actions.push_back(message_actions::LOG);
      }
      if (actions.size() >0)
      {
        converter_->callAll( actions, msg );
      }
    }
}

template<class T>
void SoundEventRegister<T>::soundCallbackMessage(std::string &key, qi::AnyValue &value, nao_interaction_msgs::AudioSourceLocalization &msg) {
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "Head";
  
  qi::AnyReferenceVector anyref;
  try{
    anyref = value.asListValuePtr();
  }
  catch(std::runtime_error& e)
  {
    ROS_DEBUG_STREAM("Could not transform AnyValue into list: " << e.what());
  }
  
  if(anyref.size() != 4) {
    ROS_DEBUG("Could not retrieve sound location");
    return;
  }
  
  qi::AnyReference ref = anyref[1].content();
  
  if(ref.kind() == qi::TypeKind_List && ref.size() == 4) {
    qi::AnyReference azi, ele, conf, ener;
    azi = ref[0].content();
    ele = ref[1].content();
    conf = ref[2].content();
    ener = ref[3].content();
    if(azi.kind() == qi::TypeKind_Float && ele.kind() == qi::TypeKind_Float
            && conf.kind() == qi::TypeKind_Float && ener.kind() == qi::TypeKind_Float) {
      msg.azimuth.data = azi.asFloat();
      msg.elevation.data = ele.asFloat();
      msg.confidence.data = conf.asFloat();
      msg.energy.data = ener.asFloat();
    } else{
      ROS_DEBUG("Could not retrieve azi/ele/conf/ener");
    }
  } else {
    ROS_DEBUG("Could not retrieve sound location");
  }
}

// http://stackoverflow.com/questions/8752837/undefined-reference-to-template-class-constructor
template class SoundEventRegister<nao_interaction_msgs::AudioSourceLocalization>;

}//namespace
