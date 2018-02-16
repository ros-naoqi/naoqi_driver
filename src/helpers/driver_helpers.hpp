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


#ifndef DRIVER_HELPERS_HPP
#define DRIVER_HELPERS_HPP

#include <naoqi_driver/tools.hpp>

#include <naoqi_bridge_msgs/RobotInfo.h>

#include <std_msgs/Empty.h>

#include <naoqi_bridge_msgs/SetString.h>

#include <naoqi_bridge_msgs/FadeLeds.h>

#include <qi/applicationsession.hpp>

namespace naoqi
{
namespace helpers
{
namespace driver
{

const robot::Robot& getRobot( const qi::SessionPtr& session );

const naoqi_bridge_msgs::RobotInfo& getRobotInfo( const qi::SessionPtr& session );

bool& setLanguage( const qi::SessionPtr& session, naoqi_bridge_msgs::SetStringRequest req );

std::string& getLanguage( const qi::SessionPtr& session );

std_msgs::Empty& fadeLeds( const qi::SessionPtr& session, naoqi_bridge_msgs::FadeLedsRequest req );

std_msgs::Empty& resetLeds( const qi::SessionPtr& session, naoqi_bridge_msgs::SetStringRequest req );

} // driver
} // helpers
} // naoqi

#endif
