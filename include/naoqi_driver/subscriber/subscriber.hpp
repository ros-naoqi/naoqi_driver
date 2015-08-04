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

#ifndef SUBSCRIBER_HPP
#define SUBSCRIBER_HPP

#include <string>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>

namespace naoqi
{
namespace subscriber
{


/**
* @brief Subscriber concept interface
* @note this defines an private concept struct,
* which each instance has to implement
* @note a type erasure pattern in implemented here to avoid strict inheritance,
* thus each possible subscriber instance has to implement the virtual functions mentioned in the concept
*/
class Subscriber
{

public:

  /**
  * @brief Constructor for subscriber interface
  */
  template<typename T>
  Subscriber( T sub ):
    subPtr_( boost::make_shared<SubscriberModel<T> >(sub) )
  {}

  /**
  * @brief checks if the subscriber is correctly initialized on the ros-master
  @ @return bool value indicating true for success
  */
  bool isInitialized() const
  {
    return subPtr_->isInitialized();
  }

  /**
  * @brief initializes/resets the subscriber into ROS with a given nodehandle,
  * this will be called at first for initialization or again when master uri has changed
  * @param ros NodeHandle to register the subscriber on
  */
  void reset( ros::NodeHandle& nh )
  {
    std::cout << name() << " is resetting" << std::endl;
    subPtr_->reset( nh );
    std::cout << name() << " reset" << std::endl;
  }

  /**
  * @brief getting the descriptive name for this subscriber instance
  * @return string with the name
  */
  std::string name() const
  {
    return subPtr_->name();
  }

  /**
  * @brief getting the topic to subscriber on
  * @return string indicating the topic
  */
  std::string topic() const
  {
    return subPtr_->topic();
  }

  friend bool operator==( const Subscriber& lhs, const Subscriber& rhs )
  {
    // decision made for OR-comparison since we want to be more restrictive
    if ( lhs.name() == rhs.name() || lhs.topic() == rhs.topic() )
      return true;
    return false;
  }

private:

  /**
  * BASE concept struct
  */
  struct SubscriberConcept
  {
    virtual ~SubscriberConcept(){}
    virtual bool isInitialized() const = 0;
    virtual void reset( ros::NodeHandle& nh ) = 0;
    virtual std::string name() const = 0;
    virtual std::string topic() const = 0;
  };


  /**
  * templated instances of base concept
  */
  template<typename T>
  struct SubscriberModel : public SubscriberConcept
  {
    SubscriberModel( const T& other ):
      subscriber_( other )
    {}

    std::string name() const
    {
      return subscriber_->name();
    }

    std::string topic() const
    {
      return subscriber_->topic();
    }

    bool isInitialized() const
    {
      return subscriber_->isInitialized();
    }

    void reset( ros::NodeHandle& nh )
    {
      subscriber_->reset( nh );
    }

    T subscriber_;
  };

  boost::shared_ptr<SubscriberConcept> subPtr_;

}; // class subscriber

} //subscriber
} //naoqi

#endif
