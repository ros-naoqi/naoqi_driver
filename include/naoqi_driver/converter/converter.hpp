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

#ifndef CONVERTER_HPP
#define CONVERTER_HPP

#include <string>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <naoqi_driver/message_actions.h>

namespace naoqi
{
namespace converter
{


/**
* @brief Converter concept interface
* @note this defines an private concept struct,
* which each instance has to implement
* @note a type erasure pattern in implemented here to avoid strict inheritance,
* thus each possible converter instance has to implement the virtual functions mentioned in the concept
*/
class Converter
{

public:

  /**
  * @brief Constructor for converter interface
  */
  template<typename T>
  Converter( T conv ):
    convPtr_( boost::make_shared<ConverterModel<T> >(conv) )
  {}

  /**
  * @brief getting the descriptive name for this converter instance
  * @return string with the name
  */
  std::string name() const
  {
    return convPtr_->name();
  }

  /**
  * @brief getting the assigned frequency of this converter instance
  * @return float value indicating the frequency
  */
  float frequency() const
  {
    return convPtr_->frequency();
  }

  void reset()
  {
    convPtr_->reset();
  }

  void callAll( const std::vector<message_actions::MessageAction>& actions )
  {
    if ( actions.size() > 0 )
    {
      convPtr_->callAll(actions);
    }

    ros::Time after = ros::Time::now();
    lapse_time = after - before;
    before = after;
  }

  ros::Duration lapseTime() const
  {
    return lapse_time;
  }

  friend bool operator==( const Converter& lhs, const Converter& rhs )
  {
    // decision made for OR-comparison since we want to be more restrictive
    if ( lhs.name() == rhs.name() )
      return true;
    return false;
  }

private:

  ros::Time before;
  ros::Duration lapse_time;

  /**
  * BASE concept struct
  */
  struct ConverterConcept
  {
    virtual ~ConverterConcept(){}
    virtual std::string name() const = 0;
    virtual float frequency() const = 0;
    virtual void reset() = 0;
    virtual void callAll( const std::vector<message_actions::MessageAction>& actions ) = 0;
  };


  /**
  * templated instances of base concept
  */
  template<typename T>
  struct ConverterModel : public ConverterConcept
  {
    ConverterModel( const T& other ):
      converter_( other )
    {}

    std::string name() const
    {
      return converter_->name();
    }

    float frequency() const
    {
      return converter_->frequency();
    }
    
    void reset()
    {
      converter_->reset();
    }

    void callAll( const std::vector<message_actions::MessageAction>& actions )
    {
      converter_->callAll( actions );
    }

    T converter_;
  };

  boost::shared_ptr<ConverterConcept> convPtr_;

}; // class converter

} //converter
} //naoqi

#endif
