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

#ifndef PUBLISHER_INFO_HPP
#define PUBLISHER_INFO_HPP

/*
* LOCAL includes
*/
#include "basic.hpp"
#include <naoqi_driver/tools.hpp>

/*
* ROS includes
*/
#include <ros/ros.h>
#include <naoqi_bridge_msgs/StringStamped.h>

namespace naoqi
{
namespace publisher
{

class InfoPublisher : public BasicPublisher<naoqi_bridge_msgs::StringStamped>
{
public:
  InfoPublisher( const std::string& topic, const robot::Robot& robot_type );

  void reset( ros::NodeHandle& nh );

  virtual inline bool isSubscribed() const
  {
    return true;
  }

protected:
  const robot::Robot& robot_;
};

} //publisher
} //naoqi

#endif
