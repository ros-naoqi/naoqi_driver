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


#ifndef TRACKER_SERVICE_HPP
#define TRACKER_SERVICE_HPP

#include <iostream>
#include <map>

#include <ros/node_handle.h>
#include <ros/service_server.h>

#include <std_srvs/Empty.h>
#include <nao_interaction_msgs/SetTrackerMode.h>
#include <nao_interaction_msgs/SetTrackerTarget.h>
#include <nao_interaction_msgs/StartTracker.h>
#include <qi/session.hpp>

namespace naoqi
{
namespace service
{

class TrackerService
{
public:
  TrackerService( const std::string& name, const std::string& topic, const qi::SessionPtr& session ):
      name_(name),
      topic_(topic),
      session_(session),
      p_tracker_(session->service("ALTracker"))
    {}

  ~TrackerService(){}

  std::string name() const
  {
    return name_;
  }

  std::string topic() const
  {
    return topic_;
  }

  virtual void reset( ros::NodeHandle& nh )=0;

protected:
  const std::string name_;
  const std::string topic_;

  const qi::SessionPtr& session_;
  qi::AnyObject p_tracker_;
  ros::ServiceServer service_;
};

class TrackerSetTargetService : public TrackerService
{
public:
  TrackerSetTargetService(const std::string& name, const std::string& topic, const qi::SessionPtr& session) : TrackerService(name, topic, session) {}
  void reset(ros::NodeHandle& nh);
  bool callback(nao_interaction_msgs::SetTrackerTargetRequest& req, nao_interaction_msgs::SetTrackerTargetResponse& resp);

};

class TrackerStartTrackingService : public TrackerService
{
public:
  TrackerStartTrackingService(const std::string& name, const std::string& topic, const qi::SessionPtr& session) : TrackerService(name, topic, session) {}
  void reset(ros::NodeHandle& nh);
  bool callback(nao_interaction_msgs::StartTrackerRequest& req, nao_interaction_msgs::StartTrackerResponse& resp);

};

class TrackerSetModeService : public TrackerService
{
public:
  TrackerSetModeService(const std::string& name, const std::string& topic, const qi::SessionPtr& session) : TrackerService(name, topic, session) {}
  void reset(ros::NodeHandle& nh);
  bool callback(nao_interaction_msgs::SetTrackerModeRequest& req, nao_interaction_msgs::SetTrackerModeResponse& resp);

};

class TrackerEmptyServices : public TrackerService
{
public:
  TrackerEmptyServices(const std::string& name, const std::string& topic, const qi::SessionPtr& session) : TrackerService(name, topic, session) {}
  void reset(ros::NodeHandle& nh);
  bool callback(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp);

};

} // service
} // naoqi
#endif
