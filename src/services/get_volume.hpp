/*
 * Copyright 2017 Aldebaran
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


#ifndef GET_VOLUME_SERVICE_HPP
#define GET_VOLUME_SERVICE_HPP

#include <iostream>

#include <ros/node_handle.h>
#include <ros/service_server.h>

#include <nao_interaction_msgs/GetAudioMasterVolume.h>
#include <qi/session.hpp>

namespace naoqi
{
namespace service
{

class GetVolumeService
{
public:
  GetVolumeService( const std::string& name, const std::string& topic, const qi::SessionPtr& session );

  ~GetVolumeService(){};

  std::string name() const
  {
    return name_;
  }
  
  std::string topic() const
  {
    return topic_;
  }

  void reset( ros::NodeHandle& nh );
  
  bool callback( nao_interaction_msgs::GetAudioMasterVolumeRequest& req, nao_interaction_msgs::GetAudioMasterVolumeResponse& resp );


private:
  const std::string name_;
  const std::string topic_;
  
  const qi::SessionPtr& session_;
  ros::ServiceServer service_;
};
  
} // service
} // naoqi
#endif
