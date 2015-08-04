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

#ifndef SONAR_CONVERTER_HPP
#define SONAR_CONVERTER_HPP

/*
* LOCAL includes
*/
#include "converter_base.hpp"
#include <naoqi_driver/message_actions.h>

/*
* ROS includes
*/
#include <sensor_msgs/Range.h>

namespace naoqi
{
namespace converter
{

class SonarConverter : public BaseConverter<SonarConverter>
{

  typedef boost::function<void(std::vector<sensor_msgs::Range>&)> Callback_t;


public:
  SonarConverter( const std::string& name, const float& frequency, const qi::SessionPtr& session );

  ~SonarConverter();

  void reset( );

  void registerCallback( message_actions::MessageAction action, Callback_t cb );

  void callAll( const std::vector<message_actions::MessageAction>& actions );


private:
  std::map<message_actions::MessageAction, Callback_t> callbacks_;

  /** Sonar (Proxy) configurations */
  qi::AnyObject p_sonar_;
  /** Memory (Proxy) configurations */
  qi::AnyObject p_memory_;
  /** Key describeing whether we are subscribed to the ALSonar module */
  bool is_subscribed_;

  /** The memory keys of the sonars */
  std::vector<std::string> keys_;
  /** The frames of the sonars */
  std::vector<std::string> frames_;
  /** Pre-filled messges that are sent */
  std::vector<sensor_msgs::Range> msgs_;
};

} //publisher
} //naoqi

#endif
