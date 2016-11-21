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

#include "tracking.hpp"
#include "../helpers/driver_helpers.hpp"

namespace naoqi
{
namespace service
{

void TrackerSetTargetService::reset( ros::NodeHandle& nh )
{
  service_ = nh.advertiseService(topic_, &TrackerSetTargetService::callback, this);
}

bool TrackerSetTargetService::callback(nao_interaction_msgs::SetTrackerTargetRequest &req, nao_interaction_msgs::SetTrackerTargetResponse &resp)
{
  if(req.target.compare(req.FACE) == 0) {
    p_tracker_.call<void>(name_, req.target, req.values[0]);
  } else if(req.target.compare(req.PEOPLE) == 0 || req.target.compare(req.SOUND) == 0) {
    p_tracker_.call<void>(name_, req.target, req.values);
  } else {
    ROS_ERROR_STREAM("Unknown target '" << req.target <<"'");
    return false;
  }
  return true;
}

void TrackerStartTrackingService::reset( ros::NodeHandle& nh )
{
  service_ = nh.advertiseService(topic_, &TrackerStartTrackingService::callback, this);
}

bool TrackerStartTrackingService::callback(nao_interaction_msgs::StartTrackerRequest& req, nao_interaction_msgs::StartTrackerResponse& resp)
{
  if(req.target.compare(req.FACE) == 0 || req.target.compare(req.PEOPLE) == 0 || req.target.compare(req.SOUND) == 0) {
    p_tracker_.call<void>(name_, req.target);
  } else {
    ROS_ERROR_STREAM("Unknown target '" << req.target <<"'");
    return false;
  }
  return true;
}

void TrackerSetModeService::reset( ros::NodeHandle& nh )
{
  service_ = nh.advertiseService(topic_, &TrackerSetModeService::callback, this);
}

bool TrackerSetModeService::callback(nao_interaction_msgs::SetTrackerModeRequest& req, nao_interaction_msgs::SetTrackerModeResponse& resp)
{
  if(req.mode.compare(req.HEAD) == 0 || req.mode.compare(req.WHOLEBODY) == 0 || req.mode.compare(req.MOVE) == 0) {
    p_tracker_.call<void>(name_, req.mode);
  } else {
    ROS_ERROR_STREAM("Unknown mode '" << req.mode <<"'");
    return false;
  }
  return true;
}

void TrackerEmptyServices::reset( ros::NodeHandle& nh )
{
  service_ = nh.advertiseService(topic_, &TrackerEmptyServices::callback, this);
}

bool TrackerEmptyServices::callback(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp)
{
  p_tracker_.call<void>(name_);
  return true;
}

}
}
