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
#include "animated_speech.hpp"


namespace naoqi
{
namespace subscriber
{

  AnimatedSpeechSubscriber::AnimatedSpeechSubscriber( const std::string& name, const std::string& animated_speech_topic, const qi::SessionPtr& session ):
    animated_speech_topic_(animated_speech_topic),
    BaseSubscriber( name, animated_speech_topic, session ),
    p_tts_( session->service("ALAnimatedSpeech") )
  {}

  void AnimatedSpeechSubscriber::reset( ros::NodeHandle& nh )
  {
    sub_animated_speech_ = nh.subscribe( animated_speech_topic_, 10, &AnimatedSpeechSubscriber::animated_speech_callback, this );

    is_initialized_ = true;
  }

  void AnimatedSpeechSubscriber::animated_speech_callback( const std_msgs::StringConstPtr& string_msg )
  {
    p_tts_.async<void>("say", string_msg->data);
  }

}// subscriber 
}// naoqi
