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

/**
* LOCAL includes
*/
#include "audio.hpp"

/**
* BOOST includes
*/
#include <boost/foreach.hpp>
#define for_each BOOST_FOREACH

namespace alros{

namespace converter{

AudioConverter::AudioConverter(const std::string& name, const float& frequency, const qi::SessionPtr& session)
    : BaseConverter(name, frequency, session),
    serviceId(0),
    p_audio_( session->service("ALAudioDevice")),
    p_robot_model_(session->service("ALRobotModel"))
{
    std::cout << "Audio Extractor: Creation" << std::endl;
    int micConfig = p_robot_model_.call<int>("_getMicrophoneConfig");
    if(micConfig){
      channelMap.push_back(3);
      channelMap.push_back(5);
      channelMap.push_back(0);
      channelMap.push_back(2);
    }
    else{
      channelMap.push_back(0);
      channelMap.push_back(2);
      channelMap.push_back(1);
      channelMap.push_back(4);
    }
}

AudioConverter::~AudioConverter() {
    stop();
}

void AudioConverter::start(){
//  std::cout << "service is number " << serviceId << std::endl;
  if(!serviceId)
  {
    serviceId = session_->registerService("ALRosBridge_Audio", shared_from_this());
    p_audio_.call<void>(
            "setClientPreferences",
            "ALRosBridge_Audio",
            48000,
            0,
            0
            );
    p_audio_.call<void>("subscribe","ALRosBridge_Audio");
    std::cout << "Audio Extractor: Start" << std::endl;
  }
}

void AudioConverter::stop(){
  if(serviceId){
    p_audio_.call<void>("unsubscribe", "ALRosBridge_Audio");
    session_->unregisterService(serviceId);
    serviceId = 0;
  }
  std::cout << "Audio Extractor: Stop" << std::endl;
}

void AudioConverter::processRemote(int nbOfChannels, int samplesByChannel, qi::AnyValue altimestamp, qi::AnyValue buffer){
    naoqi_msgs::AudioBuffer msg = naoqi_msgs::AudioBuffer();
    msg.header.stamp = ros::Time::now();
    msg.frequency = 48000;
    msg.channelMap = channelMap;

    std::pair<char*, size_t> buffer_pointer = buffer.asRaw();

    int16_t* remoteBuffer = (int16_t*)buffer_pointer.first;
    int bufferSize = nbOfChannels * samplesByChannel;
    msg.data = std::vector<int16_t>(remoteBuffer, remoteBuffer+bufferSize);

    for_each( message_actions::MessageAction action, actions_ )
    {
      callbacks_[action]( msg );
    }
}

void AudioConverter::reset()
{

}

void AudioConverter::registerCallback( const message_actions::MessageAction action, Callback_t cb )
{
  callbacks_[action] = cb;
}

void AudioConverter::callAll(const std::vector<message_actions::MessageAction>& actions)
{
  actions_ = actions;
  start();
}

}

}
