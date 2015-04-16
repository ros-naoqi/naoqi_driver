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
* STANDARD includes
*/
#include "diagnostics.hpp"

/** This file defines a Diagnostic publisher
 * It does not use the DiagnostricsUpdater for optimization.
 * A full diagnostic_msgs/DiagnosticArray is built and published
 */

namespace alros
{
namespace publisher
{

DiagnosticsPublisher::DiagnosticsPublisher( const std::string& topic ):
    BasePublisher( topic )
{
}

void DiagnosticsPublisher::publish( diagnostic_msgs::DiagnosticArray& msg )
{
  // Publish
  pub_.publish(msg);
}

void DiagnosticsPublisher::reset( ros::NodeHandle& nh)
{
  pub_ = nh.advertise< diagnostic_msgs::DiagnosticArray >( topic_, 1 );
  is_initialized_ = true;
}

} //publisher
} // alros
