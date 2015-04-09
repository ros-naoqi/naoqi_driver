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

#ifndef INT_CONVERTER_HPP
#define INT_CONVERTER_HPP

#include <map>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/Int32.h>

#include <boost/function.hpp>

#include <qi/anyobject.hpp>

#include <alrosbridge/message_actions.h>
#include "converter_base.hpp"

namespace alros
{
namespace converter
{

class IntConverter : public BaseConverter<IntConverter>
{
  typedef boost::function<void(std_msgs::Int32)> Callback_t;

public:

  IntConverter( const std::string& name, const float& frequency, const qi::SessionPtr& session );

  void reset();

  void registerCallback( const message_actions::MessageAction action, Callback_t cb );

  void callAll( const std::vector<message_actions::MessageAction>& actions );

private:

  std::map<message_actions::MessageAction, Callback_t> callbacks_;

}; // class

} //publisher
} // alros

#endif
