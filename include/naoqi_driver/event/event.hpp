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

#ifndef EVENT_HPP
#define EVENT_HPP

#include <string>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <naoqi_driver/message_actions.h>
#include <naoqi_driver/recorder/globalrecorder.hpp>
#include <naoqi_driver/tools.hpp>

namespace naoqi
{
namespace event
{


/**
* @brief Converter concept interface
* @note this defines an private concept struct,
* which each instance has to implement
* @note a type erasure pattern in implemented here to avoid strict inheritance,
* thus each possible converter instance has to implement the virtual functions mentioned in the concept
*/
class Event
{

public:

  /**
  * @brief Constructor for converter interface
  */
  template<typename T>
  Event( T event ):
    eventPtr_( boost::make_shared<EventModel<T> >(event) )
  {}

  void resetPublisher( ros::NodeHandle& nh )
  {
    eventPtr_->resetPublisher(nh);
  }

  void resetRecorder( boost::shared_ptr<naoqi::recorder::GlobalRecorder> gr )
  {
    eventPtr_->resetRecorder(gr);
  }

  void startProcess( )
  {
    eventPtr_->startProcess();
  }

  void stopProcess( )
  {
    eventPtr_->stopProcess();
  }

  void writeDump( const ros::Time& time )
  {
    eventPtr_->writeDump(time);
  }

  void setBufferDuration(float duration)
  {
    eventPtr_->setBufferDuration(duration);
  }

  void isRecording(bool state)
  {
    eventPtr_->isRecording(state);
  }

  void isPublishing(bool state)
  {
    eventPtr_->isPublishing(state);
  }

  void isDumping(bool state)
  {
    eventPtr_->isDumping(state);
  }

private:

  /**
  * BASE concept struct
  */
  struct EventConcept
  {
    virtual ~EventConcept(){}
    virtual void resetPublisher(ros::NodeHandle& nh) = 0;
    virtual void resetRecorder(boost::shared_ptr<naoqi::recorder::GlobalRecorder> gr) = 0;
    virtual void startProcess() = 0;
    virtual void stopProcess() = 0;
    virtual void writeDump(const ros::Time& time) = 0;
    virtual void setBufferDuration(float duration) = 0;
    virtual void isRecording(bool state) = 0;
    virtual void isPublishing(bool state) = 0;
    virtual void isDumping(bool state) = 0;
  };


  /**
  * templated instances of base concept
  */
  template<typename T>
  struct EventModel : public EventConcept
  {
    EventModel( const T& other ):
      converter_( other )
    {}

    void resetPublisher( ros::NodeHandle& nh )
    {
      converter_->resetPublisher(nh);
    }

    void resetRecorder( boost::shared_ptr<naoqi::recorder::GlobalRecorder> gr )
    {
      converter_->resetRecorder(gr);
    }

    void startProcess( )
    {
      converter_->startProcess();
    }

    void stopProcess( )
    {
      converter_->stopProcess();
    }

    void writeDump( const ros::Time& time )
    {
      converter_->writeDump(time);
    }

    void setBufferDuration(float duration)
    {
      converter_->setBufferDuration(duration);
    }

    void isRecording(bool state)
    {
      converter_->isRecording(state);
    }

    void isPublishing(bool state)
    {
      converter_->isPublishing(state);
    }

    void isDumping(bool state)
    {
      converter_->isDumping(state);
    }

    T converter_;
  };

  boost::shared_ptr<EventConcept> eventPtr_;

}; // class converter

} //converter
} //naoqi

#endif
