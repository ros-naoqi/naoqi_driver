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

/*
* LOCAL includes
*/
#include "sonar.hpp"

namespace naoqi
{
namespace publisher
{

SonarPublisher::SonarPublisher( const std::vector<std::string>& topics )
  : is_initialized_(false),
  topics_(topics)
{
}

void SonarPublisher::publish( const std::vector<sensor_msgs::Range>& sonar_msgs )
{
  if ( pubs_.size() != sonar_msgs.size() )
  {
    std::cerr << "Incorrect number of sonar range messages in sonar publisher. " << sonar_msgs.size() << "/" << pubs_.size() << std::endl;
    return;
  }

  for( size_t i=0; i<sonar_msgs.size(); ++i)
  {
    pubs_[i].publish( sonar_msgs[i] );
  }
}

void SonarPublisher::reset( ros::NodeHandle& nh )
{
  pubs_ = std::vector<ros::Publisher>();
  for( size_t i=0; i<topics_.size(); ++i)
  {
    pubs_.push_back( nh.advertise<sensor_msgs::Range>(topics_[i], 1) );
  }

  is_initialized_ = true;
}

} // publisher
} //naoqi
