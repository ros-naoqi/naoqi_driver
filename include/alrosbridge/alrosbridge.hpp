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
#include <alrosbridge/subscriber/subscriber.hpp>
#include <alrosbridge/recorder/recorder.hpp>

namespace alros
{

class Recorder;

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
  * @brief registers a subscriber
  * @param subscriber to register
  * @see Subscriber
  * @note it will be called by value to expose that internally there will be a copy,
  * eventually this should be replaced by move semantics C++11
  */
  void registerSubscriber( subscriber::Subscriber sub );

  /**
  * @brief qicli call function to get current master uri
  * @return string indicating http master uri
  */
  std::string getMasterURI() const;

  /**
  * @brief qicli call function to set current master uri
  * @param string in form of http://<ip>:11311
  * @param network_interface the network interface ("eth0", "tether" ...)
  */
  void setMasterURINet( const std::string& uri, const std::string& network_interface );

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

  /**
  * @brief qicli call function to start recording all registered publisher in a ROSbag
  */
  void startRecord();

  /**
  * @brief qicli call function to stop recording all registered publisher in a ROSbag
  */
  void stopRecord();

private:
  qi::SessionPtr sessionPtr_;
  bool publish_enabled_;
  bool publish_cancelled_;
  const size_t freq_;
  boost::thread publisherThread_;
  //ros::Rate r_;

  boost::shared_ptr<Recorder> _recorder;

  void registerDefaultPublisher();
  void registerDefaultSubscriber();
  void init();

  void rosLoop();


  boost::scoped_ptr<ros::NodeHandle> nhPtr_;
  boost::mutex mutex_reinit_;

  std::vector< publisher::Publisher > publishers_;
  std::vector< subscriber::Subscriber> subscribers_;

  /** Pub Publisher to execute at a specific time */
  struct ScheduledPublish {
    ScheduledPublish(const ros::Time& schedule, size_t pub_index) :
       schedule_(schedule), pub_index_(pub_index)
    {
    }

    bool operator < (const ScheduledPublish& sp_in) const {
      return schedule_ > sp_in.schedule_;
    }
    /** Time at which the publisher will be called */
    ros::Time schedule_;
    /** Time at which the publisher will be called */
    size_t pub_index_;
  };

  /** Priority queue to process the publishers according to their frequency */
  std::priority_queue<ScheduledPublish> pub_queue_;
};

} // alros

#endif
