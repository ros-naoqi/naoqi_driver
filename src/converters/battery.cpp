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
#include "battery.hpp"
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

BatteryConverter::BatteryConverter( const std::string& name, const float& frequency, const qi::SessionPtr& session )
  : BaseConverter( name, frequency, session ),
    p_memory_( session->service("ALMemory") ),
    p_battery_( session->service("ALBattery") ),
    is_subscribed_(false)
{
  std::vector<std::string> keys;
  keys.push_back("Device/SubDeviceList/Battery/Charge/Sensor/Value");
  keys.push_back("Device/SubDeviceList/Battery/Charge/Sensor/TotalVoltage");
  keys.push_back("Device/SubDeviceList/Battery/Current/Sensor/Value");
  keys.push_back("Device/SubDeviceList/Platform/ILS/Sensor/Value");
  keys.push_back("Device/SubDeviceList/Battery/Charge/Sensor/Charging");
  keys.push_back("ALBattery/ConnectedToChargingStation"); 

  keys_.resize(keys.size());
  size_t i = 0;
  for(std::vector<std::string>::const_iterator it = keys.begin(); it != keys.end(); ++it, ++i)
    keys_[i] = *it;
}

BatteryConverter::~BatteryConverter()
{
  if (is_subscribed_)
  {
    is_subscribed_ = false;
  }
}

void BatteryConverter::registerCallback( message_actions::MessageAction action, Callback_t cb )
{
  callbacks_[action] = cb;
}

void BatteryConverter::callAll( const std::vector<message_actions::MessageAction>& actions )
{
  if (!is_subscribed_)
  {
    is_subscribed_ = true;
  }

  std::vector<float> values;
  try {
      qi::AnyValue anyvalues = p_memory_.call<qi::AnyValue>("getListData", keys_);
      tools::fromAnyValueToFloatVector(anyvalues, values);
  } catch (const std::exception& e) {
    std::cerr << "Exception caught in BatteryConverter: " << e.what() << std::endl;
    return;
  }
  
  msg_.header.stamp = ros::Time::now();
  msg_.charge = (int)(values[0] * 100.0);
  msg_.voltage = values[1];
  msg_.current = values[2];
  msg_.hatch_open = (bool)values[3];
  msg_.charging = ((int)values[4]) == 8;
  msg_.connected_to_charging_station = (bool)values[5];

  for_each( message_actions::MessageAction action, actions )
  {
    callbacks_[action]( msg_ );
  }
}

void BatteryConverter::reset( )
{
  if (is_subscribed_)
  {
//    p_battery_.call<void>("unsubscribe", "ROS");
    is_subscribed_ = false;
  }
}

} // publisher
} //naoqi
