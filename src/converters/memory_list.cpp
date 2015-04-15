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
#include "memory_list.hpp"

/**
* BOOST includes
*/
#include <boost/foreach.hpp>
#define for_each BOOST_FOREACH

namespace alros {

namespace converter {

MemoryListConverter::MemoryListConverter(const std::vector<std::string>& key_list, const std::string &name, const float &frequency, const qi::SessionPtr &session):
    BaseConverter(name, frequency, session),
    p_memory_(session->service("ALMemory")),
    _key_list(key_list)
{}

void MemoryListConverter::reset(){

}

void MemoryListConverter::callAll(const std::vector<message_actions::MessageAction> &actions){
  // Get inertial data
  AL::ALValue memData = p_memory_.call<AL::ALValue>("getListData", _key_list);
  // Reset message
  _msg = naoqi_msgs::MemoryList();
  for(int i=0; i<memData.getSize(); i++){
    if(memData[i].isFloat())
    {
      naoqi_msgs::MemoryPairFloat tmp_msg;
      tmp_msg.memoryKey = _key_list[i];
      tmp_msg.data = memData[i];
      _msg.floats.push_back(tmp_msg);
    }
    else if(memData[i].isString())
    {
      naoqi_msgs::MemoryPairString tmp_msg;
      tmp_msg.memoryKey = _key_list[i];
      tmp_msg.data = memData[i].toString();
      _msg.strings.push_back(tmp_msg);
    }
    else if(memData[i].isInt())
    {
      naoqi_msgs::MemoryPairInt tmp_msg;
      tmp_msg.memoryKey = _key_list[i];
      tmp_msg.data = memData[i];
      _msg.ints.push_back(tmp_msg);
    }
    else if(memData[i].isBool())
    {
      naoqi_msgs::MemoryPairInt tmp_msg;
      tmp_msg.memoryKey = _key_list[i];
      bool value = memData[i];
      tmp_msg.data = (int) value;
      _msg.ints.push_back(tmp_msg);
    }
  }

  for_each( message_actions::MessageAction action, actions )
  {
    callbacks_[action]( _msg);
  }
}

void MemoryListConverter::registerCallback( const message_actions::MessageAction action, Callback_t cb )
{
  callbacks_[action] = cb;
}

}

}
