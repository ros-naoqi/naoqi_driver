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

#include <alrosbridge/recorder/globalrecorder.hpp>
#include <alrosbridge/message_actions.h>

namespace alros
{

template <typename Converter, typename Recorder>
EventRegister<Converter, Recorder>::EventRegister()
{
}

template <typename Converter, typename Recorder>
EventRegister<Converter, Recorder>::EventRegister( const std::string& key, const qi::SessionPtr& session )
  : key_(key),
    p_memory_( session->service("ALMemory") ),
    isStarted_(false),
    isRecording_(false),
    isDumping_(false)
{
  recorder_ = boost::make_shared<Recorder>( key_ );
  converter_ = boost::make_shared<Converter>( key_, 0, session, key_ );

  converter_->registerCallback( message_actions::RECORD, boost::bind(&Recorder::write, recorder_, _1) );
  converter_->registerCallback( message_actions::LOG, boost::bind(&Recorder::bufferize, recorder_, _1) );

  signal_ = p_memory_.call<qi::AnyObject>("subscriber", key_);
}

template <typename Converter, typename Recorder>
EventRegister<Converter, Recorder>::~EventRegister()
{
}

template <typename Converter, typename Recorder>
void EventRegister<Converter, Recorder>::resetRecorder( boost::shared_ptr<alros::recorder::GlobalRecorder> gr )
{
  recorder_->reset(gr, converter_->frequency());
}

template <typename Converter, typename Recorder>
void EventRegister<Converter, Recorder>::startProcess()
{
  boost::mutex::scoped_lock start_lock(mutex_);
  if (!isStarted_)
  {
    registerCallback();
    isStarted_ = true;
  }
}

template <typename Converter, typename Recorder>
void EventRegister<Converter, Recorder>::stopProcess()
{
  boost::mutex::scoped_lock stop_lock(mutex_);
  if (isStarted_)
  {
    unregisterCallback();
    isStarted_ = false;
  }
}

template <typename Converter, typename Recorder>
void EventRegister<Converter, Recorder>::writeDump(const ros::Time& time)
{
  if (isStarted_)
  {
    recorder_->writeDump(time);
  }
}

template <typename Converter, typename Recorder>
void EventRegister<Converter, Recorder>::setBufferDuration(float duration)
{
  recorder_->setBufferDuration(duration);
}

template <typename Converter, typename Recorder>
void EventRegister<Converter, Recorder>::isRecording(bool state)
{
  boost::mutex::scoped_lock rec_lock(mutex_);
  isRecording_ = state;
}

template <typename Converter, typename Recorder>
void EventRegister<Converter, Recorder>::isDumping(bool state)
{
  boost::mutex::scoped_lock dump_lock(mutex_);
  isDumping_ = state;
}

template <typename Converter, typename Recorder>
void EventRegister<Converter, Recorder>::registerCallback()
{
  signalID_ = signal_.connect("signal", (boost::function<void(qi::AnyValue)>(boost::bind(&EventRegister<Converter, Recorder>::onEvent,
                                                                            this))));
}

template <typename Converter, typename Recorder>
void EventRegister<Converter, Recorder>::unregisterCallback()
{
  signal_.disconnect(signalID_);
}

template <typename Converter, typename Recorder>
void EventRegister<Converter, Recorder>::onEvent()
{
  std::vector<message_actions::MessageAction> actions;
  boost::mutex::scoped_lock callback_lock(mutex_);
  if (isStarted_) {
    // CHECK FOR RECORD
    if ( isRecording_ )
    {
      actions.push_back(message_actions::RECORD);
    }
    // CHECK FOR BUFFERIZE
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
