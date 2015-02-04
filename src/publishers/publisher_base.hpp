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

#ifndef BASE_PUBLISHER_HPP
#define BASE_PUBLISHER_HPP

#include <iostream>

namespace alros
{
namespace publisher
{

// CRTP
template<class T>
class BasePublisher
{

public:
  BasePublisher( const std::string& name, const std::string& topic, float frequency ):
    name_( name ),
    topic_( topic ),
    frequency_( frequency ),
    is_initialized_( false )
  {}

  virtual ~BasePublisher() {};

  inline std::string name() const
  {
    return name_;
  }

  inline std::string topic() const
  {
    return topic_;
  }

  inline float frequency() const
  {
    return frequency_;
  }

  inline bool isInitialized() const
  {
    return is_initialized_;
  }

  virtual bool isSubscribed() const = 0;

protected:
  std::string name_, topic_;

  bool is_initialized_;

  /** Frequency at which the publisher should publish. This is informative */
  float frequency_;
}; // class

} //publisher
} // alros

#endif
