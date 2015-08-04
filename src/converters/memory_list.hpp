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

#ifndef MEMORY_LIST_CONVERTER_HPP
#define MEMORY_LIST_CONVERTER_HPP

/*
* LOCAL includes
*/
#include "converter_base.hpp"
#include <naoqi_driver/message_actions.h>

/*
* ROS includes
*/
#include <naoqi_bridge_msgs/MemoryList.h>

namespace naoqi {

namespace converter {

class MemoryListConverter : public BaseConverter<MemoryListConverter>
{
  typedef boost::function<void(naoqi_bridge_msgs::MemoryList&) > Callback_t;

public:
  MemoryListConverter(const std::vector<std::string>& key_list, const std::string& name, const float& frequency, const qi::SessionPtr& session);

  virtual void reset();

  void registerCallback( const message_actions::MessageAction action, Callback_t cb );

  virtual void callAll(const std::vector<message_actions::MessageAction>& actions );

private:
  std::vector<std::string> _key_list;
  naoqi_bridge_msgs::MemoryList _msg;
  qi::AnyObject p_memory_;
  std::vector<std::string> data_names_list_;

  /** Registered Callbacks **/
  std::map<message_actions::MessageAction, Callback_t> callbacks_;
};

}

}

#endif // MEMORY_LIST_CONVERTER_HPP
