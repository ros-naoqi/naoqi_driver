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


#ifndef LOCALIZATION_SERVICE_HPP
#define LOCALIZATION_SERVICE_HPP

#include <iostream>
#include <string>
#include <sstream>
#include <vector>

#include <ros/node_handle.h>
#include <ros/service_server.h>

#include <std_srvs/Empty.h>
#include <nao_interaction_msgs/LocalizationTrigger.h>
#include <nao_interaction_msgs/LocalizationTriggerString.h>
#include <nao_interaction_msgs/LocalizationCheck.h>
#include <nao_interaction_msgs/LocalizationGetErrorMessage.h>
#include <qi/session.hpp>

namespace naoqi
{
namespace service
{

class LocalizationService
{
public:
  LocalizationService( const std::string& name, const std::string& topic, const qi::SessionPtr& session ) :
      name_(name),
      topic_(topic),
      session_(session),
      p_localization_(session->service("ALLocalization"))
  {
    func_ = split(name_, '-').back();
  }

  ~LocalizationService(){}

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
  std::string func_;
  const std::string topic_;

  const qi::SessionPtr& session_;
  qi::AnyObject p_localization_;
  ros::ServiceServer service_;

  void split(const std::string &s, char delim, std::vector<std::string> &elems) {
      std::stringstream ss;
      ss.str(s);
      std::string item;
      while (std::getline(ss, item, delim)) {
          elems.push_back(item);
      }
  }

  std::vector<std::string> split(const std::string &s, char delim) {
      std::vector<std::string> elems;
      split(s, delim, elems);
      return elems;
  }
};

class LocalizationEmptyService : public LocalizationService
{
public:
  LocalizationEmptyService(const std::string& name, const std::string& topic, const qi::SessionPtr& session) : LocalizationService(name, topic, session) {}
  void reset(ros::NodeHandle& nh);
  bool callback(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp);

};

class LocalizationTriggerService : public LocalizationService
{
public:
  LocalizationTriggerService(const std::string& name, const std::string& topic, const qi::SessionPtr& session) : LocalizationService(name, topic, session) {}
  void reset(ros::NodeHandle& nh);
  bool callback(nao_interaction_msgs::LocalizationTriggerRequest& req, nao_interaction_msgs::LocalizationTriggerResponse& resp);

};

class LocalizationTriggerStringService : public LocalizationService
{
public:
  LocalizationTriggerStringService(const std::string& name, const std::string& topic, const qi::SessionPtr& session) : LocalizationService(name, topic, session) {}
  void reset(ros::NodeHandle& nh);
  bool callback(nao_interaction_msgs::LocalizationTriggerStringRequest& req, nao_interaction_msgs::LocalizationTriggerStringResponse& resp);

};

class LocalizationCheckService : public LocalizationService
{
public:
  LocalizationCheckService(const std::string& name, const std::string& topic, const qi::SessionPtr& session) : LocalizationService(name, topic, session) {}
  void reset(ros::NodeHandle& nh);
  bool callback(nao_interaction_msgs::LocalizationCheckRequest& req, nao_interaction_msgs::LocalizationCheckResponse& resp);

};

class LocalizationGetErrorMessageService : public LocalizationService
{
public:
  LocalizationGetErrorMessageService(const std::string& name, const std::string& topic, const qi::SessionPtr& session) : LocalizationService(name, topic, session) {}
  void reset(ros::NodeHandle& nh);
  bool callback(nao_interaction_msgs::LocalizationGetErrorMessageRequest& req, nao_interaction_msgs::LocalizationGetErrorMessageResponse& resp);

};

} // service
} // naoqi
#endif
