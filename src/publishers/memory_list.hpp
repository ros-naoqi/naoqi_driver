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


#ifndef MEMORY_LIST_PUBLISHER_HPP
#define MEMORY_LIST_PUBLISHER_HPP

/**
* ROS includes
*/
#include <naoqi_msgs/MemoryList.h>

/**
* LOCAL includes
*/
#include "publisher_base.hpp"

namespace alros {

namespace publisher {

class MemoryListPublisher: public BasePublisher<MemoryListPublisher>
{
public:
  MemoryListPublisher(const std::string& topic);

  virtual void publish( const naoqi_msgs::MemoryList& _msg);

  virtual void reset( ros::NodeHandle& nh );

};

}

}

#endif // MEMORY_LIST_PUBLISHER_HPP
