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

#include "robot_config.hpp"

namespace naoqi
{
namespace service
{

RobotConfigService::RobotConfigService( const std::string& name, const std::string& topic )
  : name_(name),
  topic_(topic)
{}

void RobotConfigService::reset( ros::NodeHandle& nh )
{
  std::cout << "i will advertise my ros service on topic " << topic_ << std::endl;
  service_ = nh.advertiseService(topic_, &RobotConfigService::callback, this);
}

bool RobotConfigService::callback( naoqi_bridge_msgs::GetRobotInfoRequest& req, naoqi_bridge_msgs::GetRobotInfoResponse& resp )
{
  std::cout << "triggering robot config service" << std::endl;
  return true;
}


}
}
