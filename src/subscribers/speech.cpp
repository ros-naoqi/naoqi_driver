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

/*
 * LOCAL includes
 */
#include "speech.hpp"


namespace naoqi
{
namespace subscriber
{

SpeechSubscriber::SpeechSubscriber( const std::string& name, const std::string& speech_topic, const qi::SessionPtr& session ):
  speech_topic_(speech_topic),
  BaseSubscriber( name, speech_topic, session ),
  p_tts_( session->service("ALTextToSpeech") )
{}

void SpeechSubscriber::reset( ros::NodeHandle& nh )
{
  sub_speech_ = nh.subscribe( speech_topic_, 10, &SpeechSubscriber::speech_callback, this );

  is_initialized_ = true;
}

void SpeechSubscriber::speech_callback( const std_msgs::StringConstPtr& string_msg )
{
  p_tts_.async<void>("say", string_msg->data);
}

} //publisher
} // naoqi
