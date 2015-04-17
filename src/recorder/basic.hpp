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

#ifndef BASIC_RECORDER_HPP
#define BASIC_RECORDER_HPP

/**
* LOCAL includes
*/
#include <alrosbridge/recorder/globalrecorder.hpp>

/**
* STANDARD includes
*/
#include <string>

namespace alros
{
namespace recorder
{

template<class T>
class BasicRecorder
{

public:
  BasicRecorder( const std::string& topic ):
    topic_( topic ),
    is_initialized_( false ),
    is_subscribed_( false )
  {}

  virtual ~BasicRecorder() {}

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

  virtual void write(const T& msg)
  {
    gr_->write(topic_, msg);
  }

  virtual void reset(boost::shared_ptr<GlobalRecorder> gr)
  {
    gr_ = gr;
    is_initialized_ = true;
  }

protected:
  std::string topic_;

  bool is_initialized_;
  bool is_subscribed_;

  boost::shared_ptr<alros::recorder::GlobalRecorder> gr_;

}; // class

} // publisher
} // alros

#endif
