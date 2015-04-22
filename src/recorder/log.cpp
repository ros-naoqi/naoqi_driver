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

#include "log.hpp"

namespace alros
{
namespace recorder
{

LogRecorder::LogRecorder( const std::string& topic ):
  topic_( topic )
{}

void LogRecorder::write(std::list<rosgraph_msgs::Log>& log_msgs)
{
  while ( !log_msgs.empty() )
  {
    if (!log_msgs.front().header.stamp.isZero()) {
      gr_->write(topic_, log_msgs.front(), log_msgs.front().header.stamp);
    }
    else {
      gr_->write(topic_, log_msgs.front());
    }
    {
      log_msgs.pop_front();
    }
  }
}

void LogRecorder::reset(boost::shared_ptr<GlobalRecorder> gr, float frequency)
{
  gr_ = gr;
  buffer_size_ = static_cast<size_t>(10*frequency);
  buffer_.resize(buffer_size_);
  is_initialized_ = true;
}

void LogRecorder::bufferize( std::list<rosgraph_msgs::Log>& log_msgs )
{
  boost::mutex::scoped_lock lock_bufferize( mutex_ );
  buffer_.pop_front();
  buffer_.push_back(log_msgs);
}

} //publisher
} // alros
