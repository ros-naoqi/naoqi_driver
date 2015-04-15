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

#ifndef MEMORY_STRING_RECORDER_HPP
#define MEMORY_STRING_RECORDER_HPP

/**
* LOCAL includes
*/
#include "../recorderbase.hpp"
#include <alrosbridge/recorder/globalrecorder.hpp>

/**
* ROS includes
*/
#include <naoqi_bridge_msgs/StringStamped.h>

namespace alros
{
namespace recorder
{

class MemoryStringRecorder : public BaseRecorder<MemoryStringRecorder>
{

public:
  MemoryStringRecorder( const std::string& topic );

  void write( const naoqi_bridge_msgs::StringStamped& msg );

  void reset( boost::shared_ptr<alros::recorder::GlobalRecorder> gr );

private:
  boost::shared_ptr<alros::recorder::GlobalRecorder> gr_;

}; // class

} //publisher
} // alros

#endif
