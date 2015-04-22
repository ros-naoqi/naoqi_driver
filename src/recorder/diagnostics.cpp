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

/**
* LOCAL includes
*/
#include "diagnostics.hpp"

namespace alros
{
namespace recorder
{

DiagnosticsRecorder::DiagnosticsRecorder( const std::string& topic ):
  topic_( topic ),
  is_initialized_( false ),
  is_subscribed_( false )
{}

void DiagnosticsRecorder::write(diagnostic_msgs::DiagnosticArray& msg)
{
  if (!msg.header.stamp.isZero()) {
    gr_->write(topic_, msg, msg.header.stamp);
  }
  else {
    gr_->write(topic_, msg);
  }
}

void DiagnosticsRecorder::writeDump()
{
  boost::mutex::scoped_lock lock_write_buffer( mutex_ );
  std::list<diagnostic_msgs::DiagnosticArray>::iterator it;
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

void DiagnosticsRecorder::reset(boost::shared_ptr<GlobalRecorder> gr, float frequency)
{
  gr_ = gr;
  buffer_size_ = static_cast<size_t>(10*frequency);
  buffer_.resize(buffer_size_);
  is_initialized_ = true;
}

void DiagnosticsRecorder::bufferize(diagnostic_msgs::DiagnosticArray& msg )
{
  boost::mutex::scoped_lock lock_bufferize( mutex_ );
  buffer_.pop_front();
  buffer_.push_back(msg);
}

} //publisher
} // alros
