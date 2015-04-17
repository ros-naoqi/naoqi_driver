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

/**
* LOCAL includes
*/
#include "diagnostics.hpp"

/**
* ROS includes
*/
#include <diagnostic_updater/DiagnosticStatusWrapper.h>

/**
* BOOST includes
*/
#include <boost/foreach.hpp>
#define for_each BOOST_FOREACH

/** This file defines a Diagnostic converter
 * It does not use the DiagnostricsUpdater for optimization.
 * A full diagnostic_msgs/DiagnosticArray is built and sent to requesting nodes
 */

namespace alros
{
namespace converter
{

DiagnosticsConverter::DiagnosticsConverter( const std::string& name, float frequency, const qi::SessionPtr& session ):
    BaseConverter( name, frequency, session ),
    p_memory_(session->service("ALMemory")),
    temperature_warn_level_(68),
    temperature_error_level_(74)
{
  // Get all the joint names
  qi::AnyObject p_motion = session->service("ALMotion");
  joint_names_ = p_motion.call<std::vector<std::string> >("getBodyNames", "JointActuators" );

  for(std::vector<std::string>::const_iterator it = joint_names_.begin(); it != joint_names_.end(); ++it)
    joint_temperatures_keys_.push_back(std::string("Device/SubDeviceList/") + (*it) + std::string("/Temperature/Sensor/Value"));

  // Get all the battery keys
  battery_keys_.push_back(std::string("Device/SubDeviceList/Battery/Charge/Sensor/Value"));
  battery_keys_.push_back(std::string("Device/SubDeviceList/Battery/Charge/Sensor/Status"));
  battery_keys_.push_back(std::string("Device/SubDeviceList/Battery/Current/Sensor/Value"));

  std::string battery_status_keys[] = {"End off Discharge flag", "Near End Off Discharge flag", "Charge FET on",
    "Discharge FET on", "Accu learn flag", "Discharging flag", "Full Charge Flag", "Charge Flag",
    "Charge Temperature Alarm", "Over Charge Alarm", "Discharge Alarm", "Charge Over Current Alarm",
    "Discharge Over Current Alarm (14A)", "Discharge Over Current Alarm (6A)", "Discharge Temperature Alarm",
    "Power-Supply present"};
  battery_status_keys_ = std::vector<std::string>(battery_status_keys, battery_status_keys+16);

  // TODO get ID from Device/DeviceList/ChestBoard/BodyId

  // Concatenate the keys in all_keys_
  all_keys_.arraySetSize(joint_temperatures_keys_.size() + battery_keys_.size());
  size_t i = 0;
  for(std::vector<std::string>::const_iterator it = joint_temperatures_keys_.begin(); it != joint_temperatures_keys_.end(); ++it, ++i)
    all_keys_[i] = *it;
  for(std::vector<std::string>::const_iterator it = battery_keys_.begin(); it != battery_keys_.end(); ++it, ++i)
    all_keys_[i] = *it;
}

void DiagnosticsConverter::callAll( const std::vector<message_actions::MessageAction>& actions )
{
  diagnostic_msgs::DiagnosticArray msg;
  msg.header.stamp = ros::Time::now();

  // Get all the keys
  //qi::details::printMetaObject(std::cout, p_memory_.metaObject());
  std::vector<float> values;
  try {
    values = p_memory_.call<std::vector<float> >("getListData", all_keys_ );
  } catch (const std::exception& e) {
    std::cerr << "Exception caught in DiagnosticsConverter: " << e.what() << std::endl;
    return;
  }

  // Fill the temperature message for the joints
  size_t val = 0;
  diagnostic_msgs::DiagnosticStatus::_level_type max_level = diagnostic_msgs::DiagnosticStatus::OK;
  for(size_t i = 0; i < joint_names_.size(); ++val, ++i)
  {
    diagnostic_updater::DiagnosticStatusWrapper status;
    status.name = std::string("/Joints/") + joint_names_[i];

    status.hardware_id = joint_names_[i];
    float temperature = float(values[val]);
    status.add("Temperature", float(values[val]));
    if (temperature < temperature_warn_level_)
    {
      status.level = diagnostic_msgs::DiagnosticStatus::OK;
      status.message = "Normal";
    }
    else if (temperature < temperature_error_level_)
    {
      status.level = diagnostic_msgs::DiagnosticStatus::WARN;
      status.message = "Hot";
    }
    else
    {
      status.level = diagnostic_msgs::DiagnosticStatus::ERROR;
      status.message = "Too hot";
    }

    max_level = std::max(max_level, status.level);
    msg.status.push_back(status);
  }

  // Get the aggregated joints
  {
    diagnostic_updater::DiagnosticStatusWrapper status;
    status.name = std::string("/Joints");
    status.hardware_id = "joints";
    status.level = max_level;

    msg.status.push_back(status);
  }

  // Fill the message for the battery
  {
    int battery_percentage = 100.0 * float(values[val++]);
    int battery_status = int(float(values[val++]));

    diagnostic_updater::DiagnosticStatusWrapper status;
    status.name = std::string("/Battery/Battery");
    status.hardware_id = "battery";
    status.add("Percentage", battery_percentage);
    for( size_t i = 0; i < battery_status_keys_.size(); ++i)
      status.add(battery_status_keys_[i], bool((1 << i) && battery_status));
    //status.add("Status", std::string());

    // TODO, convert battery_status to a string of binary 0101010
    status.add("Status", battery_status);

    std::ostringstream ss;
    if (bool((1 << 6) && battery_status))
    {
      status.level = diagnostic_msgs::DiagnosticStatus::OK;
      ss << "Battery fully charged";
    }
    else if (bool((1 << 7) && battery_status))
    {
      status.level = diagnostic_msgs::DiagnosticStatus::OK;
      ss << "Charging (" << std::setw(4) << battery_percentage << "%%)";
    }
    else
    {
      if (battery_percentage > 60)
      {
        status.level = diagnostic_msgs::DiagnosticStatus::OK;
        ss << "Battery OK (" << std::setw(4) << battery_percentage << "%% left)";
      }
      else if (battery_percentage > 30)
      {
        status.level = diagnostic_msgs::DiagnosticStatus::WARN;
        ss << "Battery discharging (" << std::setw(4) << battery_percentage << "%% left)";
      }
      else
      {
        status.level = diagnostic_msgs::DiagnosticStatus::ERROR;
        ss << "Battery almost empty (" << std::setw(4) << battery_percentage << "%% left)";
      }
    }
    status.message = ss.str();

    max_level = status.level;
    msg.status.push_back(status);
  }

  // Process the current information
  {
    float current = float(values[val++]);
    diagnostic_updater::DiagnosticStatusWrapper status;
    status.name = std::string("/Battery/Current");
    status.hardware_id = "battery";
    status.add("Current", current);
    status.level = max_level;
    std::ostringstream ss;
    if (current > 0)
      ss << "Total Current: " << std::setw(5) << current << " Ampere (charging)";
    else
      ss << "Total Current: " << std::setw(5) << current << " Ampere (discharging)";
    status.message = ss.str();

    msg.status.push_back(status);
  }

  // Get the aggregated battery
  {
    diagnostic_updater::DiagnosticStatusWrapper status;
    status.name = std::string("/Battery");
    status.hardware_id = "battery";
    status.level = diagnostic_msgs::DiagnosticStatus::OK;

    msg.status.push_back(status);
  }

  // TODO: CPU information should be obtained from system files like done in Python

  // TODO: wifi and ethernet statuses should be obtained from DBUS

  for_each( message_actions::MessageAction action, actions )
  {
    callbacks_[action]( msg);
  }

}

void DiagnosticsConverter::reset()
{
}

void DiagnosticsConverter::registerCallback( const message_actions::MessageAction action, Callback_t cb )
{
  callbacks_[action] = cb;
}

} //converter
} // alros
