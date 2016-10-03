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
#include "play_animation.hpp"


namespace naoqi
{
namespace subscriber
{

PlayAnimationSubscriber::PlayAnimationSubscriber( const std::string& name, const std::string& pa_topic, const qi::SessionPtr& session ):
  pa_topic_(pa_topic),
  BaseSubscriber( name, pa_topic, session ),
  p_animation_player_( session->service("ALAnimationPlayer") )
{}

void PlayAnimationSubscriber::reset( ros::NodeHandle& nh )
{
  sub_pa_ = nh.subscribe( pa_topic_, 10, &PlayAnimationSubscriber::pa_callback, this );

  is_initialized_ = true;
}

void PlayAnimationSubscriber::pa_callback( const std_msgs::StringConstPtr& string_msg )
{
  if(string_msg->data.find("/") != std::string::npos)
    p_animation_player_.async<void>("run", string_msg->data);
  else
    p_animation_player_.async<void>("runTag", string_msg->data);
}

} //publisher
} // naoqi
