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

#include "localization.hpp"
#include "../helpers/driver_helpers.hpp"

namespace naoqi
{
namespace service
{

void LocalizationEmptyService::reset( ros::NodeHandle& nh )
{
  service_ = nh.advertiseService(topic_, &LocalizationEmptyService::callback, this);
}

bool LocalizationEmptyService::callback(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp)
{
  p_localization_.call<void>(func_);
  return true;
}

// ##############

void LocalizationTriggerService::reset( ros::NodeHandle& nh )
{
  service_ = nh.advertiseService(topic_, &LocalizationTriggerService::callback, this);
}

bool LocalizationTriggerService::callback(nao_interaction_msgs::LocalizationTriggerRequest& req, nao_interaction_msgs::LocalizationTriggerResponse& resp)
{
  resp.result = p_localization_.call<int>(func_);
  return true;
}

// ##############

void LocalizationTriggerStringService::reset( ros::NodeHandle& nh )
{
  service_ = nh.advertiseService(topic_, &LocalizationTriggerStringService::callback, this);
}

bool LocalizationTriggerStringService::callback(nao_interaction_msgs::LocalizationTriggerStringRequest& req, nao_interaction_msgs::LocalizationTriggerStringResponse& resp)
{
  resp.result = p_localization_.call<int>(func_, req.value);
  return true;
}

// ##############

void LocalizationCheckService::reset( ros::NodeHandle& nh )
{
  service_ = nh.advertiseService(topic_, &LocalizationCheckService::callback, this);
}

bool LocalizationCheckService::callback(nao_interaction_msgs::LocalizationCheckRequest& req, nao_interaction_msgs::LocalizationCheckResponse& resp)
{
  resp.result = p_localization_.call<bool>(func_);
  return true;
}

// ##############

void LocalizationGetErrorMessageService::reset( ros::NodeHandle& nh )
{
  service_ = nh.advertiseService(topic_, &LocalizationGetErrorMessageService::callback, this);
}

bool LocalizationGetErrorMessageService::callback(nao_interaction_msgs::LocalizationGetErrorMessageRequest& req, nao_interaction_msgs::LocalizationGetErrorMessageResponse& resp)
{
  resp.error_message = p_localization_.call<std::string>(func_, req.error_code);
  return true;
}

}
}
