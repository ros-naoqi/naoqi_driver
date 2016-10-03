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


#ifndef PLAY_ANIMATION_SUBSCRIBER_HPP
#define PLAY_ANIMATION_SUBSCRIBER_HPP

/*
 * LOCAL includes
 */
#include "subscriber_base.hpp"

/*
 * ROS includes
 */
#include <ros/ros.h>
#include <std_msgs/String.h>

namespace naoqi
{
namespace subscriber
{

class PlayAnimationSubscriber: public BaseSubscriber<PlayAnimationSubscriber>
{
public:
  PlayAnimationSubscriber( const std::string& name, const std::string& pa_topic, const qi::SessionPtr& session );
  ~PlayAnimationSubscriber(){}

  void reset( ros::NodeHandle& nh );
  void pa_callback( const std_msgs::StringConstPtr& speech_msg );

private:

  std::string pa_topic_;

  qi::AnyObject p_animation_player_;
  ros::Subscriber sub_pa_;



}; // class Speech

} // subscriber
}// naoqi
#endif
