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


#ifndef NAVIGATION_SERVICE_HPP
#define NAVIGATION_SERVICE_HPP

#include <iostream>
#include <map>

#include <ros/node_handle.h>
#include <ros/service_server.h>

#include <std_srvs/Empty.h>
#include <nao_interaction_msgs/GoToPose.h>
#include <qi/session.hpp>
#include <tf2_ros/buffer.h>

namespace naoqi
{
namespace service
{

class NavigationService
{
public:
  NavigationService( const std::string& name, const std::string& topic, const qi::SessionPtr& session ):
      name_(name),
      topic_(topic),
      session_(session),
      p_navigation_(session->service("ALNavigation"))
    {
      func_ = split(name_, '-').back();
    }

  ~NavigationService(){}

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
  qi::AnyObject p_navigation_;
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

class NavigationEmptyService : public NavigationService
{
public:
  NavigationEmptyService(const std::string& name, const std::string& topic, const qi::SessionPtr& session) : NavigationService(name, topic, session) {}
  void reset(ros::NodeHandle& nh);
  bool callback(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp);

};

class NavigateToService : public NavigationService
{
public:
  NavigateToService(const std::string& name, const std::string& topic, const qi::SessionPtr& session, const boost::shared_ptr<tf2_ros::Buffer>& tf2_buffer) : NavigationService(name, topic, session), tf2_buffer_(tf2_buffer) {}
  void reset(ros::NodeHandle& nh);
  bool callback(nao_interaction_msgs::GoToPoseRequest& req, nao_interaction_msgs::GoToPoseResponse& resp);

private:
  boost::shared_ptr<tf2_ros::Buffer> tf2_buffer_;

};

} // service
} // naoqi
#endif
