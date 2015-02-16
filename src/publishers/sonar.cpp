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

#include <alvalue/alvalue.h>

#include <ros/serialization.h>

#include "sonar.hpp"

namespace alros
{
namespace publisher
{

SonarPublisher::SonarPublisher( const std::string& name, const std::string& topic, float frequency, qi::SessionPtr& session )
  : BasePublisher( name, topic, frequency, session ),
    p_memory_( session->service("ALMemory") ),
    p_sonar_( session->service("ALSonar") ),
    is_subscribed_(false)
{
  std::vector<std::string> keys;
  if (robot() == PEPPER) {
    keys.push_back("Device/SubDeviceList/Platform/Front/Sonar/Sensor/Value");
    keys.push_back("Device/SubDeviceList/Platform/Back/Sonar/Sensor/Value");
    frames_.push_back("SonarFront_frame");
    frames_.push_back("SonarBack_frame");
    topics_.push_back(topic + "/Front_sensor");
    topics_.push_back(topic + "/Back_sensor");
  } else if (robot() == NAO) {
    keys.push_back("Device/SubDeviceList/US/Left/Sensor/Value");
    keys.push_back("Device/SubDeviceList/US/Right/Sensor/Value");
    frames_.push_back("LSonar_frame");
    frames_.push_back("RSonar_frame");
    topics_.push_back(topic + "/Left_sensor");
    topics_.push_back(topic + "/Right_sensor");
  }

  // Prepare the messages
  msgs_.resize(frames_.size());
  for(size_t i = 0; i < msgs_.size(); ++i)
    {
      msgs_[i].header.frame_id = frames_[i];
      msgs_[i].min_range = 0.25;
      msgs_[i].max_range = 2.55;
      msgs_[i].field_of_view = 0.523598776;
      msgs_[i].radiation_type = sensor_msgs::Range::ULTRASOUND;
    }

  keys_.arraySetSize(keys.size());
  size_t i = 0;
  for(std::vector<std::string>::const_iterator it = keys.begin(); it != keys.end(); ++it, ++i)
    keys_[i] = *it;
}

SonarPublisher::~SonarPublisher()
{
  if (is_subscribed_)
  {
    p_sonar_.call<AL::ALValue>("unsubscribe", "ROS");
    is_subscribed_ = false;
  }
}

void SonarPublisher::publish()
{
  if (!is_subscribed_)
  {
    p_sonar_.call<AL::ALValue>("subscribe", "ROS");
    is_subscribed_ = true;
  }

  AL::ALValue values = p_memory_.call<AL::ALValue>("getListData", keys_);
  ros::Time now = ros::Time::now();
  for(size_t i = 0; i < msgs_.size(); ++i)
  {
    msgs_[i].header.stamp = now;
    msgs_[i].range = float(values[i]);
    pubs_[i].publish(msgs_[i]);
  }
}

void SonarPublisher::reset( ros::NodeHandle& nh )
{
  if (is_subscribed_)
  {
    p_sonar_.call<AL::ALValue>("unsubscribe", "ROS");
    is_subscribed_ = false;
  }

  pubs_.clear();
  if (true) {
    for(std::vector<std::string>::const_iterator it = topics_.begin(); it != topics_.end(); ++it)
      pubs_.push_back( nh.advertise<sensor_msgs::Range>( *it, 1 ) );
  }

  is_initialized_ = true;
}

} // publisher
} //alros
