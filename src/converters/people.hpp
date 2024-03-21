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

#ifndef PEOPLE_CONVERTER_HPP
#define PEOPLE_CONVERTER_HPP

/*
* LOCAL includes
*/
#include "converter_base.hpp"
#include <naoqi_driver/message_actions.h>

/*
* ROS includes
*/
#include <people_msgs/People.h>
#include <people_msgs/Person.h>
#include <geometry_msgs/Point.h>

namespace naoqi
{
namespace converter
{

class PeopleConverter : public BaseConverter<PeopleConverter>
{

  typedef boost::function<void(people_msgs::People&)> Callback_t;

public:
  PeopleConverter( const std::string& name, const float& frequency, const qi::SessionPtr& session );

  void registerCallback( message_actions::MessageAction action, Callback_t cb );

  void callAll( const std::vector<message_actions::MessageAction>& actions );

  void reset( );

private:

  qi::AnyObject p_memory_;

  std::map<message_actions::MessageAction, Callback_t> callbacks_;

  people_msgs::People msg_;

}; // class

} //publisher
} // naoqi

#endif
