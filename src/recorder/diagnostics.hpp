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

#ifndef DIAGNOSTICS_RECORDER_HPP
#define DIAGNOSTICS_RECORDER_HPP

/**
* LOCAL includes
*/
#include <alrosbridge/recorder/globalrecorder.hpp>

/**
* ROS includes
*/
#include <diagnostic_msgs/DiagnosticArray.h>

namespace alros
{
namespace recorder
{

class DiagnosticsRecorder
{

public:
  DiagnosticsRecorder( const std::string& topic );

  void write(diagnostic_msgs::DiagnosticArray& msg );

  void reset( boost::shared_ptr<alros::recorder::GlobalRecorder> gr, float frequency );

  inline std::string topic() const
  {
    return topic_;
  }

  inline bool isInitialized() const
  {
    return is_initialized_;
  }

  inline void subscribe( bool state)
  {
    is_subscribed_ = state;
  }

  inline bool isSubscribed() const
  {
    return is_subscribed_;
  }

protected:
  std::string topic_;

  bool is_initialized_;
  bool is_subscribed_;

  boost::shared_ptr<alros::recorder::GlobalRecorder> gr_;

}; // class

} //publisher
} // alros

#endif
