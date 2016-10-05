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


#ifndef BEHAVIOR_MANAGER_SERVICE_HPP
#define BEHAVIOR_MANAGER_SERVICE_HPP

#include <iostream>

#include <ros/node_handle.h>
#include <ros/service_server.h>

#include <nao_interaction_msgs/BehaviorManagerInfo.h>
#include <nao_interaction_msgs/BehaviorManagerControl.h>
#include <qi/session.hpp>

namespace naoqi
{
namespace service
{

class BehaviorManagerService
{
public:
  BehaviorManagerService( const std::string& name, const std::string& topic, const qi::SessionPtr& session ) :
      name_(name),
      topic_(topic),
      session_(session),
      p_bm_(session->service("ALBehaviorManager")) {}

  ~BehaviorManagerService(){};

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
  qi::AnyObject p_bm_;
  ros::ServiceServer service_;
};

class BehaviorManagerInfoService : public BehaviorManagerService
{
public:
  BehaviorManagerInfoService(const std::string& name, const std::string& topic, const qi::SessionPtr& session) : BehaviorManagerService(name, topic, session) {}
  void reset(ros::NodeHandle& nh);
  bool callback(nao_interaction_msgs::BehaviorManagerInfoRequest& req, nao_interaction_msgs::BehaviorManagerInfoResponse& resp);

};

class BehaviorManagerControlService : public BehaviorManagerService
{
public:
  BehaviorManagerControlService(const std::string& name, const std::string& topic, const qi::SessionPtr& session) : BehaviorManagerService(name, topic, session) {}
  void reset(ros::NodeHandle& nh);
  bool callback(nao_interaction_msgs::BehaviorManagerControlRequest& req, nao_interaction_msgs::BehaviorManagerControlResponse& resp);

};

} // service
} // naoqi
#endif
