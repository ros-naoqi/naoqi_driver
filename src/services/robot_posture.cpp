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

#include "robot_posture.hpp"
#include "../helpers/driver_helpers.hpp"

namespace naoqi
{
namespace service
{

void RobotPostureEmptyService::reset( ros::NodeHandle& nh )
{
  service_ = nh.advertiseService(topic_, &RobotPostureEmptyService::callback, this);
}

bool RobotPostureEmptyService::callback(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp)
{
  p_posture_.call<void>(func_);
  return true;
}

// ##############

void RobotPostureGoToService::reset( ros::NodeHandle& nh )
{
  service_ = nh.advertiseService(topic_, &RobotPostureGoToService::callback, this);
}

bool RobotPostureGoToService::callback(nao_interaction_msgs::GoToPostureRequest& req, nao_interaction_msgs::GoToPostureResponse& resp)
{
  if(req.posture_name.compare(req.STAND_INIT) == 0 || req.posture_name.compare(req.STAND_ZERO) == 0 || req.posture_name.compare(req.CROUCH) == 0) {
    p_posture_.call<int>(func_, req.posture_name, req.speed);
  } else {
    ROS_ERROR_STREAM("Unknown posture '" << req.posture_name <<"'");
    return false;
  }
  return true;
}

}
}
