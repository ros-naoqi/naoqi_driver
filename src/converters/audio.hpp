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

#ifndef AUDIO_CONVERTER_HPP
#define AUDIO_CONVERTER_HPP

/**
* LOCAL includes
*/
#include "converter_base.hpp"
#include <alrosbridge/message_actions.h>

/**
* BOOST includes
*/
#include <boost/enable_shared_from_this.hpp>

/**
* ROS includes
*/
#include <naoqi_msgs/AudioBuffer.h>

/**
* ALDEBARAN includes
*/
#include <qi/anymodule.hpp>

namespace alros{

namespace converter{

class AudioConverter : public BaseConverter<AudioConverter>, public boost::enable_shared_from_this<AudioConverter>
{

  typedef boost::function<void(naoqi_msgs::AudioBuffer&) > Callback_t;

public:
  AudioConverter(const std::string& name, const float& frequency, const qi::SessionPtr& session);

  ~AudioConverter();

  void processRemote(int nbOfChannels, int samplesByChannel, AL::ALValue altimestamp, AL::ALValue buffer);

  virtual void reset();

  void registerCallback( const message_actions::MessageAction action, Callback_t cb );

  virtual void callAll(const std::vector<message_actions::MessageAction>& actions);

private:
  void start();
  void stop();

  qi::AnyObject p_audio_;
  qi::AnyObject p_robot_model_;
  qi::FutureSync<qi::AnyObject> p_audio_extractor_request;
  std::vector<uint8_t> channelMap;
  unsigned int serviceId;
  /** Registered Callbacks **/
  std::map<message_actions::MessageAction, Callback_t> callbacks_;
  std::vector<message_actions::MessageAction> actions_;
};

QI_REGISTER_OBJECT(AudioConverter, processRemote)

}

}

#endif // AUDIO_CONVERTER_HPP
