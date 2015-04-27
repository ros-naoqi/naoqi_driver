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

#ifndef SONAR_RECORDER_HPP
#define SONAR_RECORDER_HPP

/**
* LOCAL includes
*/
#include <alrosbridge/recorder/globalrecorder.hpp>

/**
* ROS includes
*/
#include <sensor_msgs/Range.h>

namespace alros
{
namespace recorder
{

class SonarRecorder
{

public:
  SonarRecorder( const std::vector<std::string>& topics, float buffer_frequency = 0 );

  void write(const std::vector<sensor_msgs::Range>& sonar_msgs );

  void reset( boost::shared_ptr<alros::recorder::GlobalRecorder> gr, float conv_frequency );

  void bufferize(const std::vector<sensor_msgs::Range>& sonar_msgs );

  void writeDump();

  void setBufferDuration(float duration);

  inline std::string topic() const
  {
    return "sonar";
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

  std::list< std::vector<sensor_msgs::Range> > buffer_;
  size_t buffer_size_;
  float buffer_duration_;

  boost::mutex mutex_;

  bool is_initialized_;
  bool is_subscribed_;

  boost::shared_ptr<alros::recorder::GlobalRecorder> gr_;
  std::vector<std::string> topics_;

  float buffer_frequency_;
  int counter_;
  int max_counter_;

}; // class

} //publisher
} // alros

#endif
