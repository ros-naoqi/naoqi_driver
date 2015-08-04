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

/*
* LOCAL includes
*/
#include "sonar.hpp"
#include "../tools/from_any_value.hpp"

/*
* BOOST includes
*/
#include <boost/foreach.hpp>
#define for_each BOOST_FOREACH

namespace naoqi
{
namespace converter
{

SonarConverter::SonarConverter( const std::string& name, const float& frequency, const qi::SessionPtr& session )
  : BaseConverter( name, frequency, session ),
    p_memory_( session->service("ALMemory") ),
    p_sonar_( session->service("ALSonar") ),
    is_subscribed_(false)
{
  std::vector<std::string> keys;
  if (robot_ == robot::PEPPER) {
    keys.push_back("Device/SubDeviceList/Platform/Front/Sonar/Sensor/Value");
    keys.push_back("Device/SubDeviceList/Platform/Back/Sonar/Sensor/Value");
    frames_.push_back("SonarFront_frame");
    frames_.push_back("SonarBack_frame");
    //topics_.push_back(topic + "/Front_sensor");
    //topics_.push_back(topic + "/Back_sensor");
  } else if (robot_ == robot::NAO) {
    keys.push_back("Device/SubDeviceList/US/Left/Sensor/Value");
    keys.push_back("Device/SubDeviceList/US/Right/Sensor/Value");
    frames_.push_back("LSonar_frame");
    frames_.push_back("RSonar_frame");
    //topics_.push_back(topic + "/Left_sensor");
    //topics_.push_back(topic + "/Right_sensor");
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

  keys_.resize(keys.size());
  size_t i = 0;
  for(std::vector<std::string>::const_iterator it = keys.begin(); it != keys.end(); ++it, ++i)
    keys_[i] = *it;
}

SonarConverter::~SonarConverter()
{
  if (is_subscribed_)
  {
    p_sonar_.call<void>("unsubscribe", "ROS");
    is_subscribed_ = false;
  }
}

void SonarConverter::registerCallback( message_actions::MessageAction action, Callback_t cb )
{
  callbacks_[action] = cb;
}

void SonarConverter::callAll( const std::vector<message_actions::MessageAction>& actions )
{
  if (!is_subscribed_)
  {
    p_sonar_.call<void>("subscribe", "ROS");
    is_subscribed_ = true;
  }

  std::vector<float> values;
  try {
      qi::AnyValue anyvalues = p_memory_.call<qi::AnyValue>("getListData", keys_);
      tools::fromAnyValueToFloatVector(anyvalues, values);
  } catch (const std::exception& e) {
    std::cerr << "Exception caught in SonarConverter: " << e.what() << std::endl;
    return;
  }
  ros::Time now = ros::Time::now();
  for(size_t i = 0; i < msgs_.size(); ++i)
  {
    msgs_[i].header.stamp = now;
    msgs_[i].range = float(values[i]);
  }

  for_each( message_actions::MessageAction action, actions )
  {
    callbacks_[action]( msgs_ );
  }
}

void SonarConverter::reset( )
{
  if (is_subscribed_)
  {
    p_sonar_.call<void>("unsubscribe", "ROS");
    is_subscribed_ = false;
  }
}

} // publisher
} //naoqi
