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
#include "laser.hpp"
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

static const char* laserMemoryKeys[] = {
  // RIGHT LASER
  "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg01/X/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg01/Y/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg02/X/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg02/Y/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg03/X/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg03/Y/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg04/X/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg04/Y/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg05/X/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg05/Y/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg06/X/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg06/Y/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg07/X/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg07/Y/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg08/X/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg08/Y/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg09/X/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg09/Y/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg10/X/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg10/Y/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg11/X/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg11/Y/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg12/X/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg12/Y/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg13/X/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg13/Y/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg14/X/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg14/Y/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg15/X/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg15/Y/Sensor/Value",
  // FRONT LASER
  "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg01/X/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg01/Y/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg02/X/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg02/Y/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg03/X/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg03/Y/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg04/X/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg04/Y/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg05/X/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg05/Y/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg06/X/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg06/Y/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg07/X/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg07/Y/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg08/X/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg08/Y/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg09/X/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg09/Y/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg10/X/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg10/Y/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg11/X/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg11/Y/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg12/X/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg12/Y/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg13/X/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg13/Y/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg14/X/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg14/Y/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg15/X/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg15/Y/Sensor/Value",
  // LEFT LASER
  "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg01/X/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg01/Y/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg02/X/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg02/Y/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg03/X/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg03/Y/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg04/X/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg04/Y/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg05/X/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg05/Y/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg06/X/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg06/Y/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg07/X/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg07/Y/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg08/X/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg08/Y/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg09/X/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg09/Y/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg10/X/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg10/Y/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg11/X/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg11/Y/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg12/X/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg12/Y/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg13/X/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg13/Y/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg14/X/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg14/Y/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg15/X/Sensor/Value",
  "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg15/Y/Sensor/Value",
};

LaserConverter::LaserConverter( const std::string& name, const float& frequency, const qi::SessionPtr& session ):
  BaseConverter( name, frequency, session ),
  p_memory_(session->service("ALMemory"))
{
}

void LaserConverter::registerCallback( message_actions::MessageAction action, Callback_t cb )
{
  callbacks_[action] = cb;
}

void LaserConverter::callAll( const std::vector<message_actions::MessageAction>& actions )
{
  static const std::vector<std::string> laser_keys_value(laserMemoryKeys, laserMemoryKeys+90);

  std::vector<float> result_value;
  try {
      qi::AnyValue anyvalues = p_memory_.call<qi::AnyValue>("getListData", laser_keys_value);
      tools::fromAnyValueToFloatVector(anyvalues, result_value);
  } catch (const std::exception& e) {
    std::cerr << "Exception caught in LaserConverter: " << e.what() << std::endl;
    return;
  }
  msg_.header.stamp = ros::Time::now();
  //  prepare the right sensor frame
  size_t pos = 0;

  /**
   * there are two things done here:
   * 1.) we have to reorder the array indices
   * since there are ordered from left-to-right, ros laserscans are ordered from
   * 2.) in order to combine all lasers into one message,
   * we transform (statically) from laser frame into base_footprint
   */
  // ranges between 0-29
  for( size_t i=0; i<30; i=i+2, ++pos)
  {
    const float lx = result_value[28-i]; // segments are internally flipped
    const float ly = result_value[28-i+1]; // segments are internally flipped
    float bx = lx*std::cos(-1.757) - ly*std::sin(-1.757) - 0.018;
    float by = lx*std::sin(-1.757) + ly*std::cos(-1.757) - 0.090;
    float dist = std::sqrt( std::pow(bx,2) + std::pow(by,2) );
    //float dist = std::sqrt( std::pow(lx,2) + std::pow(ly,2) );
    //std::cout << "got a distance at "<< pos << " with "  << dist << std::endl;
    msg_.ranges[pos] = dist;
  }

  pos = pos+8; // leave out 8 blanks ==> pos = 15+8

  // ranges between 30-59
  for( size_t i=0; i<30; i=i+2, ++pos)
  {
    const float lx = result_value[58-i];
    const float ly = result_value[58-i+1];
    float bx = lx + 0.056 ;
    float by = ly;
    float dist = std::sqrt( std::pow(bx,2) + std::pow(by,2) );
    //float dist = std::sqrt( std::pow(lx,2) + std::pow(ly,2) );
    //std::cout << "got a distance at "<< pos << " with "  << dist << std::endl;
    msg_.ranges[pos] = dist;
  }

  pos = pos+8; // leave out again 8 blanks ==> pos = 15+8+15+8

  for( size_t i=0; i<30; i=i+2, ++pos)
  {
    const float lx = result_value[88-i];
    const float ly = result_value[88-i+1];
    float bx = lx*std::cos(1.757) - ly*std::sin(1.757) - 0.018;
    float by = lx*std::sin(1.757) + ly*std::cos(1.757) + 0.090;
    float dist = std::sqrt( std::pow(bx,2) + std::pow(by,2) );
    //float dist = std::sqrt( std::pow(lx,2) + std::pow(ly,2) );
    //std::cout << "got a distance at "<< pos << " with "  << dist << std::endl;
    msg_.ranges[pos] = dist;
  }

  for_each( message_actions::MessageAction action, actions )
  {
    callbacks_[action](msg_);
  }
}

void LaserConverter::reset( )
{
  msg_.header.frame_id = "base_footprint";
  msg_.angle_min = -2.0944;   // -120
  msg_.angle_max = 2.0944;    // +120
  msg_.angle_increment = (2*2.0944) / (15+15+15+8+8); // 240 deg FoV / 61 points (blind zones inc)
  msg_.range_min = 0.1; // in m
  msg_.range_max = 1.5; // in m
  msg_.ranges = std::vector<float>(61, -1.0f);
}

} //converter
} // naoqi
