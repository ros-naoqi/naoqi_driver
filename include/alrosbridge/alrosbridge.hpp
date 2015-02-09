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


#ifndef ALROS_BRIDGE_HPP
#define ALROS_BRIDGE_HPP

#include <vector>
#include <queue>

/*
* BOOST
*/
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/scoped_ptr.hpp>

/*
* ALDEB
*/
#include <qi/session.hpp>

/*
* PUBLIC INTERFACE
*/
#include <alrosbridge/publisher/publisher.hpp>

namespace alros
{


/**
* @brief Interface for ALRosBridge which is registered as a naoqi2 Module,
* once the external roscore ip is set, this class will advertise and publish ros messages
*/
class Bridge
{
public:
  /**
  * @brief Constructor for ALRosBridge
  * @param session[in] session pointer for naoqi2 service registration
  */
  Bridge( qi::SessionPtr& session );

  /**
  * @brief Destructor for ALRosBridge,
  * destroys all ros nodehandle and shutsdown all publisher
  */
  ~Bridge();

  /**
  * @brief registers a publisher
  * @param publisher to register
  * @see Publisher
  * @note it will be called by value to expose that internally there will be a copy,
  * eventually this should be replaced by move semantics C++11
  */
  void registerPublisher( publisher::Publisher pub );

  /**
  * @brief poll function to check if publishing is enabled
  * @return bool indicating if publishing is enabled/disabled
  */
  bool isAlive() const;

  /**
  * @brief qicli call function to get current master uri
  * @return string indicating http master uri
  */
  std::string getMasterURI() const;

  /**
  * @brief qicli call function to set current master uri
  * @param string in form of http://<ip>:11311
  */
  void setMasterURI( const std::string& uri );

  /**
  * @brief qicli call function to start/enable publishing all registered publisher
  */
  void start();

  /**
  * @brief qicli call function to stop/disable publishing all registered publisher
  */
  void stop();

private:
  qi::SessionPtr sessionPtr_;
  bool publish_enabled_;
  const size_t freq_;
  boost::thread publisherThread_;
  //ros::Rate r_;

  void registerDefaultPublisher();
  void initPublisher();

  void rosLoop();


  boost::scoped_ptr<ros::NodeHandle> nhPtr_;
  boost::mutex mutex_reinit_;

  std::vector< publisher::Publisher > all_publisher_;

  /** Pub Publisher to execute at a specific time */
  struct ScheduledPublish {
    ScheduledPublish(const ros::Time& schedule, alros::publisher::Publisher* pub) :
       schedule_(schedule), pub_(pub)
    {
    }

    ScheduledPublish& operator= ( const ScheduledPublish& rhs )
    {
      schedule_ = rhs.schedule_;

      alros::publisher::Publisher*& ptr= const_cast< alros::publisher::Publisher*& >(this->pub_);
      ptr = rhs.pub_;

      //alros::publisher::Publisher** ptr= const_cast< alros::publisher::Publisher** >(&(this->pub_));
      //*ptr = rhs.pub_;
      return *this;
    }

    bool operator < (const ScheduledPublish& sp_in) const {
      return schedule_ > sp_in.schedule_;
    }
    /** Time at which the publisher will be called */
    ros::Time schedule_;
    /** Time at which the publisher will be called */
    alros::publisher::Publisher* const pub_;

    ScheduledPublish(const ScheduledPublish& rhs):schedule_(rhs.schedule_), pub_(rhs.pub_) {}
  };

  /** Priority queue to process the publishers according to their frequency */
  std::priority_queue<ScheduledPublish> pub_queue_;
};

} // alros

#endif
