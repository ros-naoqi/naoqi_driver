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

#ifndef BASIC_PUBLISHER_HPP
#define BASIC_PUBLISHER_HPP

#include <string>

/*
* ROS includes
*/
#include <ros/ros.h>

namespace naoqi
{
namespace publisher
{

template<class T>
class BasicPublisher
{

public:
  BasicPublisher( const std::string& topic ):
    topic_( topic ),
    is_initialized_( false )
  {}

  virtual ~BasicPublisher() {}

  inline std::string topic() const
  {
    return topic_;
  }

  inline bool isInitialized() const
  {
    return is_initialized_;
  }

  virtual inline bool isSubscribed() const
  {
    if (is_initialized_ == false) return false;
      return pub_.getNumSubscribers() > 0;
  }

  virtual void publish( const T& msg )
  {
    pub_.publish( msg );
  }

  virtual void reset( ros::NodeHandle& nh )
  {
    pub_ = nh.advertise<T>( this->topic_, 10 );
    is_initialized_ = true;
  }

protected:
  std::string topic_;

  bool is_initialized_;

  /** Publisher */
  ros::Publisher pub_;
}; // class

} // publisher
} // naoqi

#endif
