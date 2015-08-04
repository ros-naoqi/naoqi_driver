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

namespace naoqi
{

template <typename Converter, typename Publisher, typename Recorder>
EventRegister<Converter, Publisher, Recorder>::EventRegister()
{
}

template <typename Converter, typename Publisher, typename Recorder>
EventRegister<Converter, Publisher, Recorder>::EventRegister( const std::string& key, const qi::SessionPtr& session )
  : key_(key),
    p_memory_( session->service("ALMemory") ),
    isStarted_(false),
    isPublishing_(false),
    isRecording_(false),
    isDumping_(false)
{
  publisher_ = boost::make_shared<Publisher>( key_ );
  recorder_ = boost::make_shared<Recorder>( key_ );
  converter_ = boost::make_shared<Converter>( key_, 0, session, key_ );

  converter_->registerCallback( message_actions::PUBLISH, boost::bind(&Publisher::publish, publisher_, _1) );
  converter_->registerCallback( message_actions::RECORD, boost::bind(&Recorder::write, recorder_, _1) );
  converter_->registerCallback( message_actions::LOG, boost::bind(&Recorder::bufferize, recorder_, _1) );

  signal_ = p_memory_.call<qi::AnyObject>("subscriber", key_);
}

template <typename Converter, typename Publisher, typename Recorder>
EventRegister<Converter, Publisher, Recorder>::~EventRegister()
{
}

template <typename Converter, typename Publisher, typename Recorder>
void EventRegister<Converter, Publisher, Recorder>::resetPublisher(  ros::NodeHandle& nh )
{
  publisher_->reset(nh);
}

template <typename Converter, typename Publisher, typename Recorder>
void EventRegister<Converter, Publisher, Recorder>::resetRecorder( boost::shared_ptr<naoqi::recorder::GlobalRecorder> gr )
{
  recorder_->reset(gr, converter_->frequency());
}

template <typename Converter, typename Publisher, typename Recorder>
void EventRegister<Converter, Publisher, Recorder>::startProcess()
{
  boost::mutex::scoped_lock start_lock(mutex_);
  if (!isStarted_)
  {
    registerCallback();
    isStarted_ = true;
  }
}

template <typename Converter, typename Publisher, typename Recorder>
void EventRegister<Converter, Publisher, Recorder>::stopProcess()
{
  boost::mutex::scoped_lock stop_lock(mutex_);
  if (isStarted_)
  {
    unregisterCallback();
    isStarted_ = false;
  }
}

template <typename Converter, typename Publisher, typename Recorder>
void EventRegister<Converter, Publisher, Recorder>::writeDump(const ros::Time& time)
{
  if (isStarted_)
  {
    recorder_->writeDump(time);
  }
}

template <typename Converter, typename Publisher, typename Recorder>
void EventRegister<Converter, Publisher, Recorder>::setBufferDuration(float duration)
{
  recorder_->setBufferDuration(duration);
}

template <typename Converter, typename Publisher, typename Recorder>
void EventRegister<Converter, Publisher, Recorder>::isRecording(bool state)
{
  boost::mutex::scoped_lock rec_lock(mutex_);
  isRecording_ = state;
}

template <typename Converter, typename Publisher, typename Recorder>
void EventRegister<Converter, Publisher, Recorder>::isPublishing(bool state)
{
  boost::mutex::scoped_lock pub_lock(mutex_);
  isPublishing_ = state;
}

template <typename Converter, typename Publisher, typename Recorder>
void EventRegister<Converter, Publisher, Recorder>::isDumping(bool state)
{
  boost::mutex::scoped_lock dump_lock(mutex_);
  isDumping_ = state;
}

template <typename Converter, typename Publisher, typename Recorder>
void EventRegister<Converter, Publisher, Recorder>::registerCallback()
{
  signalID_ = signal_.connect("signal", (boost::function<void(qi::AnyValue)>(boost::bind(&EventRegister<Converter, Publisher, Recorder>::onEvent,
                                                                            this))));
}

template <typename Converter, typename Publisher, typename Recorder>
void EventRegister<Converter, Publisher, Recorder>::unregisterCallback()
{
  signal_.disconnect(signalID_);
}

template <typename Converter, typename Publisher, typename Recorder>
void EventRegister<Converter, Publisher, Recorder>::onEvent()
{
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
      converter_->callAll( actions );
    }
  }
}

}//namespace
