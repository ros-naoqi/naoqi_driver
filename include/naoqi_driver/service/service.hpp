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

#ifndef SERVICE_HPP
#define SERVICE_HPP

#include <string>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>

namespace naoqi
{
namespace service
{


/**
* @brief Service concept interface
* @note this defines an private concept struct,
* which each instance has to implement
* @note a type erasure pattern in implemented here to avoid strict inheritance,
* thus each possible service instance has to implement the virtual functions mentioned in the concept
*/
class Service
{

public:

  /**
  * @brief Constructor for service interface
  */
  template<typename T>
  Service( T srv ):
    srvPtr_( boost::make_shared<ServiceModel<T> >(srv) )
  {}

  /**
  * @brief initializes/resets the service into ROS with a given nodehandle,
  * this will be called at first for initialization or again when master uri has changed
  * @param ros NodeHandle to register the service on
  */
  void reset( ros::NodeHandle& nh )
  {
    std::cout << name() << " is resetting" << std::endl;
    srvPtr_->reset( nh );
  }

  /**
  * @brief getting the descriptive name for this service instance
  * @return string with the name
  */
  std::string name() const
  {
    return srvPtr_->name();
  }

  /**
  * @brief getting the topic to service on
  * @return string indicating the topic
  */
  std::string topic() const
  {
    return srvPtr_->topic();
  }

private:

  /**
  * BASE concept struct
  */
  struct ServiceConcept
  {
    virtual ~ServiceConcept(){}
    virtual void reset( ros::NodeHandle& nh ) = 0;
    virtual std::string name() const = 0;
    virtual std::string topic() const = 0;
  };


  /**
  * templated instances of base concept
  */
  template<typename T>
  struct ServiceModel : public ServiceConcept
  {
    ServiceModel( const T& other ):
      service_( other )
    {}

    std::string name() const
    {
      return service_->name();
    }

    std::string topic() const
    {
      return service_->topic();
    }

    bool isInitialized() const
    {
      return service_->isInitialized();
    }

    void reset( ros::NodeHandle& nh )
    {
      service_->reset( nh );
    }

    T service_;
  };

  boost::shared_ptr<ServiceConcept> srvPtr_;

}; // class service

} //service
} //naoqi

#endif
