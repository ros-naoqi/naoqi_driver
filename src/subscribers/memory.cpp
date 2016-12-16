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
#include "memory.hpp"


namespace naoqi
{
namespace subscriber
{

MemorySubscriber::MemorySubscriber( const std::string& name, const std::string& memory_topic, const qi::SessionPtr& session ):
  memory_topic_(memory_topic),
  BaseSubscriber( name, memory_topic, session ),
  p_memory_( session->service("ALMemory") )
{}

void MemorySubscriber::reset( ros::NodeHandle& nh )
{
  sub_memory_ = nh.subscribe( memory_topic_, 10, &MemorySubscriber::memory_callback, this );

  is_initialized_ = true;
}

void MemorySubscriber::memory_callback( const naoqi_bridge_msgs::MemoryListConstPtr &msg )
{
  std::vector<qi::AnyValue> memData;
  for(int i = 0; i < msg->strings.size(); i++){
    std::vector<qi::AnyValue> insert;
    insert.push_back(qi::AnyValue::from(msg->strings[i].memoryKey));
    insert.push_back(qi::AnyValue::from(msg->strings[i].data));

    memData.push_back(qi::AnyValue::from(insert));
  }

  for(int i = 0; i < msg->ints.size(); i++){
    std::vector<qi::AnyValue> insert;
    insert.push_back(qi::AnyValue::from(msg->ints[i].memoryKey));
    insert.push_back(qi::AnyValue::from(msg->ints[i].data));

    memData.push_back(qi::AnyValue::from(insert));
  }

  for(int i = 0; i < msg->floats.size(); i++){
    std::vector<qi::AnyValue> insert;
    insert.push_back(qi::AnyValue::from(msg->floats[i].memoryKey));
    insert.push_back(qi::AnyValue::from(msg->floats[i].data));

    memData.push_back(qi::AnyValue::from(insert));
  }

  p_memory_.async<void>("insertListData", qi::AnyValue::from(memData));
}

} //publisher
} // naoqi
