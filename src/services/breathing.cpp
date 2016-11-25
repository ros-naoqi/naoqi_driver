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

#include "breathing.hpp"
#include "../helpers/driver_helpers.hpp"

namespace naoqi
{
namespace service
{

void EnableBreathingService::reset( ros::NodeHandle& nh )
{
  service_ = nh.advertiseService(topic_, &EnableBreathingService::callback, this);
}

bool EnableBreathingService::callback(nao_interaction_msgs::SetBreathEnabledRequest& req, nao_interaction_msgs::SetBreathEnabledResponse& resp)
{
  if(req.chain_name.compare(req.HEAD) == 0
          || req.chain_name.compare(req.BODY) == 0
          || req.chain_name.compare(req.ARMS) == 0
          || req.chain_name.compare(req.LEGS) == 0
          || req.chain_name.compare(req.LARM) == 0
          || req.chain_name.compare(req.RARM) == 0) {
    p_motion_.call<void>(name_, req.chain_name, req.enable);
  } else {
    ROS_ERROR_STREAM("Unknown chain_name '" << req.chain_name <<"'");
    return false;
  }
  return true;
}

}
}
