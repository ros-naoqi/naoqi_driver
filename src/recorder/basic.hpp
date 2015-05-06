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

/*
* LOCAL includes
*/
#include <alrosbridge/recorder/globalrecorder.hpp>
#include "../helpers.hpp"

/*
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
  BasicRecorder( const std::string& topic, float buffer_frequency = 0 ):
    topic_( topic ),
    buffer_duration_( helpers::bufferDefaultDuration ),
    is_initialized_( false ),
    is_subscribed_( false ),
    buffer_frequency_(buffer_frequency)
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
    if (!msg.header.stamp.isZero()) {
      gr_->write(topic_, msg, msg.header.stamp);
    }
    else {
      gr_->write(topic_, msg);
    }
  }

  virtual void writeDump()
  {
    boost::mutex::scoped_lock lock_write_buffer( mutex_ );
    typename std::list<T>::iterator it;
    for (it = buffer_.begin(); it != buffer_.end(); it++)
    {
      if (!it->header.stamp.isZero()) {
        gr_->write(topic_, *it, it->header.stamp);
      }
      else {
        gr_->write(topic_, *it);
      }
    }
  }

  virtual void bufferize(const T& msg)
  {
    boost::mutex::scoped_lock lock_bufferize( mutex_ );
    if (counter_ < max_counter_)
    {
      counter_++;
    }
    else
    {
      counter_ = 1;
      buffer_.pop_front();
      buffer_.push_back(msg);
    }
  }

  virtual void reset(boost::shared_ptr<GlobalRecorder> gr, float conv_frequency)
  {
    gr_ = gr;
    if (buffer_frequency_ != 0)
    {
      max_counter_ = static_cast<int>(conv_frequency/buffer_frequency_);
      buffer_size_ = static_cast<size_t>(buffer_duration_*(conv_frequency/max_counter_));
    }
    else
    {
      max_counter_ = 1;
      buffer_size_ = static_cast<size_t>(buffer_duration_*conv_frequency);
    }
    buffer_.resize(buffer_size_);
    is_initialized_ = true;
  }

  virtual void setBufferDuration(float duration)
  {
    boost::mutex::scoped_lock lock_bufferize( mutex_ );
    buffer_size_ = ( buffer_size_ / buffer_duration_ ) * duration;
    buffer_duration_ = duration;
    buffer_.resize(buffer_size_);
  }

protected:
  std::string topic_;

  std::list<T> buffer_;
  size_t buffer_size_;
  float buffer_duration_;
  boost::mutex mutex_;

  bool is_initialized_;
  bool is_subscribed_;

  boost::shared_ptr<alros::recorder::GlobalRecorder> gr_;

  float buffer_frequency_;
  int counter_;
  int max_counter_;

}; // class

} // publisher
} // alros

#endif
