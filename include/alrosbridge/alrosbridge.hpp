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
#include <boost/property_tree/ptree.hpp>
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
#include <alrosbridge/converter/converter.hpp>
#include <alrosbridge/publisher/publisher.hpp>
#include <alrosbridge/subscriber/subscriber.hpp>
#include <alrosbridge/recorder/recorder.hpp>
#include <alrosbridge/recorder/globalrecorder.hpp>

namespace tf2_ros
{
  class Buffer;
}

namespace alros
{

namespace recorder
{
  class GlobalRecorder;
}
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
   * @brief registers generall converter units
   * they are connected via callbacks to various actions such as record, log, publish
   */
  void registerConverter( converter::Converter& conv );

  /**
   * @brief prepare and register a publisher
   * @param conv_name the name of the converter related to the publisher
   * @param pub       the publisher to add
   */
  void registerPublisher( const std::string& conv_name, publisher::Publisher& pub);

  /**
   * @brief prepare and register a recorder
   * @param conv_name the name of the converter related to the recorder
   * @param rec       the recorder to add
   */
  void registerRecorder( const std::string& conv_name, recorder::Recorder& rec);

  /**
   * @brief register a converter with an associated publisher and recorder
   */
  void registerConverter(converter::Converter conv, publisher::Publisher pub, recorder::Recorder rec );

  /**
   * @brief register a converter with an associated publisher instance
   */
  void registerPublisher(converter::Converter conv, publisher::Publisher pub );

  /**
   * @brief register a converter with an associated recorder instance
   */
  void registerRecorder(converter::Converter conv, recorder::Recorder rec );

  /**
   * @brief register a converter for a given memory key
   */
  void registerMemoryConverter(const std::string& key, float frequency, const dataType::DataType& type );

  /**
   * @brief get all available converters
   */
  std::vector<std::string> getAvailableConverters();

  /**
   * @brief get all subscribed publishers
   */
  std::vector<std::string> getSubscribedPublishers() const;

  std::string _whoIsYourDaddy()
  {
    return "ask surya";
  }

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
  void startPublishing();

  /**
  * @brief qicli call function to stop/disable publishing all registered publisher
  */
  void stopPublishing();

  /**
  * @brief qicli call function to start recording all registered converter in a ROSbag
  */
  void startRecording();

  /**
  * @brief qicli call function to start recording given topics in a ROSbag
  */
  void startRecordingConverters(const std::vector<std::string>& names);

  /**
  * @brief qicli call function to stop recording all registered publisher in a ROSbag
  */
  std::string stopRecording();

  /**
   * @brief qicli call function to add on-the-fly some memory keys extractors
   */
  void addMemoryConverters(std::string filepath);

  void parseJsonFile(std::string filepath, boost::property_tree::ptree& pt);

  void stopService();

private:
  qi::SessionPtr sessionPtr_;
  bool publish_enabled_;
  bool record_enabled_;
  bool keep_looping;

  const size_t freq_;
  boost::thread publisherThread_;
  //ros::Rate r_;

  boost::shared_ptr<recorder::GlobalRecorder> recorder_;

  void registerDefaultConverter();
  void registerDefaultSubscriber();

  template <typename T1, typename T2, typename T3>
  void _registerMemoryConverter( const std::string& key, float frequency ) {
    boost::shared_ptr<T1> mfp = boost::make_shared<T1>( key );
    boost::shared_ptr<T2> mfr = boost::make_shared<T2>( key );
    boost::shared_ptr<T3> mfc = boost::make_shared<T3>( key , frequency, sessionPtr_, key );
    mfc->registerCallback( message_actions::PUBLISH, boost::bind(&T1::publish, mfp, _1) );
    mfc->registerCallback( message_actions::RECORD, boost::bind(&T2::write, mfr, _1) );
    registerConverter( mfc, mfp, mfr );
  }

  void rosLoop();
  void startRosLoop();
  void stopRosLoop();

  dataType::DataType getDataType(const std::string& key);

  boost::scoped_ptr<ros::NodeHandle> nhPtr_;
  boost::mutex mutex_reinit_;
  boost::mutex mutex_conv_queue_;
  boost::mutex mutex_record_;

  std::vector< converter::Converter > converters_;
  std::map< std::string, publisher::Publisher > pub_map_;
  std::map< std::string, recorder::Recorder > rec_map_;
  typedef std::map< std::string, publisher::Publisher>::const_iterator PubConstIter;
  typedef std::map< std::string, publisher::Publisher>::iterator PubIter;
  typedef std::map< std::string, recorder::Recorder>::const_iterator RecConstIter;
  typedef std::map< std::string, recorder::Recorder>::iterator RecIter;

  std::vector< subscriber::Subscriber > subscribers_;

  /** Pub Publisher to execute at a specific time */
  struct ScheduledConverter {
    ScheduledConverter(const ros::Time& schedule, size_t conv_index) :
       schedule_(schedule), conv_index_(conv_index)
    {
    }

    bool operator < (const ScheduledConverter& sp_in) const {
      return schedule_ > sp_in.schedule_;
    }
    /** Time at which the publisher will be called */
    ros::Time schedule_;
    /** Time at which the publisher will be called */
    size_t conv_index_;
  };

  /** Priority queue to process the publishers according to their frequency */
  std::priority_queue<ScheduledConverter> conv_queue_;

  /** tf2 buffer that will be shared between different publishers/subscribers
   * This is only for performance improvements
   */
  boost::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
};

} // alros

#endif
