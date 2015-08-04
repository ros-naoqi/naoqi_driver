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

#ifndef PUBLISHER_HPP
#define PUBLISHER_HPP

#include <string>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>

namespace naoqi
{
namespace publisher
{


/**
* @brief Publisher concept interface
* @note this defines an private concept struct,
* which each instance has to implement
* @note a type erasure pattern in implemented here to avoid strict inheritance,
* thus each possible publisher instance has to implement the virtual functions mentioned in the concept
*/
class Publisher
{

public:

  /**
  * @brief Constructor for publisher interface
  */
  template<typename T>
  Publisher( const T& pub ):
    pubPtr_( boost::make_shared<PublisherModel<T> >(pub) )
  {}

  /**
  * @brief checks if the publisher is correctly initialized on the ros-master
  @ @return bool value indicating true for success
  */
  bool isInitialized() const
  {
    return pubPtr_->isInitialized();
  }

  /**
  * @brief checks if the publisher has a subscription and is hence allowed to publish
  * @return bool value indicating true for number of sub > 0
  */
  bool isSubscribed() const
  {
    return pubPtr_->isSubscribed();
  }

  /**
  * @brief initializes/resets the publisher into ROS with a given nodehandle,
  * this will be called at first for initialization or again when master uri has changed
  * @param ros NodeHandle to advertise the publisher on
  */
  void reset( ros::NodeHandle& nh )
  {
    std::cout << topic() << " is resetting" << std::endl;
    pubPtr_->reset( nh );
    std::cout << topic() << " reset" << std::endl;
  }

  /**
  * @brief getting the topic to publish on
  * @return string indicating the topic
  */
  std::string topic() const
  {
    return pubPtr_->topic();
  }

  friend bool operator==( const Publisher& lhs, const Publisher& rhs )
  {
    // decision made for OR-comparison since we want to be more restrictive
    if ( lhs.topic() == rhs.topic() )
      return true;
    return false;
  }

  friend bool operator==( const boost::shared_ptr<Publisher>& lhs, const boost::shared_ptr<Publisher>& rhs )
  {
    return operator==( *lhs, *rhs );
  }

private:

  /**
  * BASE concept struct
  */
  struct PublisherConcept
  {
    virtual ~PublisherConcept(){}
    virtual bool isInitialized() const = 0;
    virtual bool isSubscribed() const = 0;
    virtual void reset( ros::NodeHandle& nh ) = 0;
    virtual std::string topic() const = 0;
  };


  /**
  * templated instances of base concept
  */
  template<typename T>
  struct PublisherModel : public PublisherConcept
  {
    PublisherModel( const T& other ):
      publisher_( other )
    {}

    std::string topic() const
    {
      return publisher_->topic();
    }

    bool isInitialized() const
    {
      return publisher_->isInitialized();
    }

    bool isSubscribed() const
    {
      return publisher_->isSubscribed();
    }

    void reset( ros::NodeHandle& nh )
    {
      publisher_->reset( nh );
    }

    T publisher_;
  };

  boost::shared_ptr<PublisherConcept> pubPtr_;

}; // class publisher

} //publisher
} //naoqi

#endif
