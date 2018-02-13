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

#include <sstream>
#include <string>
/*
* LOCAL includes
*/
#include "people.hpp"
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

static const char* peopleListMemoryKey = "PeoplePerception/PeopleList";

static const std::string personPositionMemoryKey[] = {
  "PeoplePerception/Person/",
  "/PositionInTorsoFrame"
};



PeopleConverter::PeopleConverter( const std::string& name, const float& frequency, const qi::SessionPtr& session ):
  BaseConverter( name, frequency, session ),
  p_memory_(session->service("ALMemory"))
{

}

void PeopleConverter::registerCallback( message_actions::MessageAction action, Callback_t cb )
{
  callbacks_[action] = cb;
}

void PeopleConverter::callAll( const std::vector<message_actions::MessageAction>& actions )
{

  std::vector<int> people_ids;

  msg_.people = std::vector<people_msgs::Person>();
  // Retreive all id's of people.
  try {
    qi::AnyValue anyvalues = p_memory_.call<qi::AnyValue>("getData", peopleListMemoryKey);
    tools::fromAnyValueToIntVector(anyvalues, people_ids);
  } catch (const std::exception& e) {
    std::cerr << "Exception caught in PersonConverter: " << e.what() << std::endl;
    return;
  }
  // Loop over all ids
  for_each(unsigned int id, people_ids)
  {
    people_msgs::Person person;
    //Set person name to id
    person.name = std::to_string(id);

    std::vector<float> position;

    try {
      //Construct memory key from id and constant parts
      std::stringstream ss;
      ss << personPositionMemoryKey[0] << id << personPositionMemoryKey[1];
      //Getreive data from ALMemory
      qi::AnyValue anyvalues = p_memory_.call<qi::AnyValue>("getData", ss.str());

      tools::fromAnyValueToFloatVector(anyvalues, position);
      // Set Person position to position relative to robot.
      person.position.x = position[0];
      person.position.y = position[1];
      person.position.z = position[2];
      // Add person to the People message
      msg_.people.push_back(person);

    } catch (const std::exception& e) {
      std::cerr << "Exception caught in PersonConverter: " << e.what() << std::endl;
      return;
    }

  }

  // Set header timestamp
  msg_.header.stamp = ros::Time::now();

  for_each( message_actions::MessageAction action, actions )
  {
    callbacks_[action](msg_);
  }
}

  void PeopleConverter::reset( )
  {
    msg_.header.frame_id = "torso";
    msg_.people = std::vector<people_msgs::Person>();
  }

} //converter

} // naoqi
