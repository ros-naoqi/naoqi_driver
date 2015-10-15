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

#ifndef TOUCH_EVENT_REGISTER_HPP
#define TOUCH_EVENT_REGISTER_HPP

#include <string>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/enable_shared_from_this.hpp>

#include <qi/session.hpp>

#include <ros/ros.h>
#include <naoqi_bridge_msgs/Bumper.h>
#include <naoqi_bridge_msgs/TactileTouch.h>

#include <naoqi_driver/tools.hpp>
#include <naoqi_driver/recorder/globalrecorder.hpp>

// Converter
#include "../src/converters/touch.hpp"
// Publisher
#include "../src/publishers/basic.hpp"
// Recorder
#include "../recorder/basic_event.hpp"

namespace naoqi
{

/**
* @brief GlobalRecorder concept interface
* @note this defines an private concept struct,
* which each instance has to implement
* @note a type erasure pattern in implemented here to avoid strict inheritance,
* thus each possible publisher instance has to implement the virtual functions mentioned in the concept
*/
class BumperEventRegister: public boost::enable_shared_from_this<BumperEventRegister>
{

public:

  /**
  * @brief Constructor for recorder interface
  */
  BumperEventRegister();
  BumperEventRegister(const std::string& name, const std::vector<std::string> keys, const float& frequency, const qi::SessionPtr& session );
  ~BumperEventRegister();

  void resetPublisher( ros::NodeHandle& nh );
  void resetRecorder( boost::shared_ptr<naoqi::recorder::GlobalRecorder> gr );

  void startProcess();
  void stopProcess();

  void writeDump(const ros::Time& time);
  void setBufferDuration(float duration);

  void isRecording(bool state);
  void isPublishing(bool state);
  void isDumping(bool state);

  void touchCallback(std::string &key, qi::AnyValue &value, qi::AnyValue &message);

private:
  void registerCallback();
  void unregisterCallback();
  void onEvent();

private:
  boost::shared_ptr<converter::TouchEventConverter<naoqi_bridge_msgs::Bumper> > converter_;
  boost::shared_ptr<publisher::BasicPublisher<naoqi_bridge_msgs::Bumper> > publisher_;
  //boost::shared_ptr<recorder::BasicEventRecorder<naoqi_bridge_msgs::Bumper> > recorder_;

  qi::SessionPtr session_;
  qi::AnyObject p_touch_;
  qi::AnyObject p_memory_;
  unsigned int serviceId;
  std::vector<std::string> keys_;
  std::string name_;

  boost::mutex mutex_;

  bool isStarted_;
  bool isPublishing_;
  bool isRecording_;
  bool isDumping_;

}; // class


class TactileTouchEventRegister: public boost::enable_shared_from_this<TactileTouchEventRegister>
{

public:

  /**
  * @brief Constructor for recorder interface
  */
  TactileTouchEventRegister();
  TactileTouchEventRegister(const std::string& name, const std::vector<std::string> keys, const float& frequency, const qi::SessionPtr& session );
  ~TactileTouchEventRegister();

  void resetPublisher( ros::NodeHandle& nh );
  void resetRecorder( boost::shared_ptr<naoqi::recorder::GlobalRecorder> gr );

  void startProcess();
  void stopProcess();

  void writeDump(const ros::Time& time);
  void setBufferDuration(float duration);

  void isRecording(bool state);
  void isPublishing(bool state);
  void isDumping(bool state);

  void touchCallback(std::string &key, qi::AnyValue &value, qi::AnyValue &message);

private:
  void registerCallback();
  void unregisterCallback();
  void onEvent();

private:
  boost::shared_ptr<converter::TouchEventConverter<naoqi_bridge_msgs::TactileTouch> > converter_;
  boost::shared_ptr<publisher::BasicPublisher<naoqi_bridge_msgs::TactileTouch> > publisher_;
  //boost::shared_ptr<recorder::BasicEventRecorder<naoqi_bridge_msgs::TactileTouch> > recorder_;

  qi::SessionPtr session_;
  qi::AnyObject p_touch_;
  qi::AnyObject p_memory_;
  unsigned int serviceId;
  std::vector<std::string> keys_;
  std::string name_;

  boost::mutex mutex_;

  bool isStarted_;
  bool isPublishing_;
  bool isRecording_;
  bool isDumping_;

}; // class


} //naoqi

#endif
