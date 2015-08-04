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
#include "bool.hpp"

/*
* BOOST includes
*/
#include <boost/foreach.hpp>
#define for_each BOOST_FOREACH

namespace naoqi
{
namespace converter
{

MemoryBoolConverter::MemoryBoolConverter( const std::string& name, const float& frequency, const qi::SessionPtr& session, const std::string& memory_key )
  : BaseConverter( name, frequency, session ),
    memory_key_(memory_key),
    p_memory_( session->service("ALMemory") )
{}

void MemoryBoolConverter::registerCallback( message_actions::MessageAction action, Callback_t cb )
{
  callbacks_[action] = cb;
}

bool MemoryBoolConverter::convert()
{
  bool success = false;
  try {
    bool value = p_memory_.call<bool>("getData", memory_key_);
    msg_.header.stamp = ros::Time::now();
    msg_.data = value;
    success = true;
  } catch (const std::exception& e) {
    std::cerr << "Exception caught in MemoryBoolConverter: " << e.what() << std::endl;
  }
  return success;
}

void MemoryBoolConverter::callAll( const std::vector<message_actions::MessageAction>& actions )
{
  if (convert()) {
    for_each( message_actions::MessageAction action, actions )
    {
      callbacks_[action]( msg_ );
    }
  }
}

void MemoryBoolConverter::reset( )
{}

} // publisher
} //naoqi
