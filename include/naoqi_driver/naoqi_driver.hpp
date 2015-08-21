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


#ifndef NAOQI_DRIVER_HPP
#define NAOQI_DRIVER_HPP

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
#include <naoqi_driver/converter/converter.hpp>
#include <naoqi_driver/publisher/publisher.hpp>
#include <naoqi_driver/subscriber/subscriber.hpp>
#include <naoqi_driver/service/service.hpp>
#include <naoqi_driver/recorder/recorder.hpp>
#include <naoqi_driver/event/event.hpp>
#include <naoqi_driver/recorder/globalrecorder.hpp>

namespace tf2_ros
{
  class Buffer;
}

namespace naoqi
{

namespace recorder
{
  class GlobalRecorder;
}
/**
* @brief Interface for naoqi driver which is registered as a naoqi2 Module,
* once the external roscore ip is set, this class will advertise and publish ros messages
*/
class Driver
{
public:
  /**
  * @brief Constructor for naoqi driver
  * @param session[in] session pointer for naoqi2 service registration
  */
  Driver( qi::SessionPtr& session, const std::string& prefix );

  /**
  * @brief Destructor for naoqi driver,
  * destroys all ros nodehandle and shutsdown all publisher
  */
  ~Driver();

  void init();

  void startRosLoop();
  void stopRosLoop();
  /**
   * @brief Write a ROSbag with the last bufferized data (10s by default)
   */
  std::string minidump(const std::string& prefix);
  std::string minidumpConverters(const std::string& prefix, const std::vector<std::string>& names);

  void setBufferDuration(float duration);
  float getBufferDuration();

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
  void registerRecorder(const std::string& conv_name, recorder::Recorder& rec, float frequency);

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
   * @brief qicli call function to register a converter for a given memory key
   */
  bool registerMemoryConverter(const std::string& key, float frequency, const dataType::DataType& type );

  /**
   * @brief qicli call function to register a converter for a given memory event
   */
  bool registerEventConverter(const std::string& key, const dataType::DataType& type);


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
    return "the coolest German in Paris";
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
   * @brief registers a service
   * @param service to register
   * @see Service
   * @note it iwll be called by value to expose that internally there will be a copy,
   * eventually this should be replaced by move semantics C++11
   */
  void registerService( service::Service srv );
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

  void startLogging();

  void stopLogging();

  /**
   * @brief qicli call function to add on-the-fly some memory keys extractors
   */
  void addMemoryConverters(std::string filepath);

  void parseJsonFile(std::string filepath, boost::property_tree::ptree& pt);

  void stopService();

  std::vector<std::string> getFilesList();

  void removeAllFiles();

  void removeFiles(std::vector<std::string> files);

private:
  qi::SessionPtr sessionPtr_;

  const robot::Robot& robot_;

  bool publish_enabled_;
  bool record_enabled_;
  bool log_enabled_;
  bool keep_looping;

  const size_t freq_;
  boost::thread publisherThread_;
  //ros::Rate r_;

  boost::shared_ptr<recorder::GlobalRecorder> recorder_;

  /* boot config */
  boost::property_tree::ptree boot_config_;
  void loadBootConfig();

  void registerDefaultConverter();
  void registerDefaultSubscriber();
  void registerDefaultServices();
  void insertEventConverter(const std::string& key, event::Event event);

  template <typename T1, typename T2, typename T3>
  void _registerMemoryConverter( const std::string& key, float frequency ) {
    boost::shared_ptr<T1> mfp = boost::make_shared<T1>( key );
    boost::shared_ptr<T2> mfr = boost::make_shared<T2>( key );
    boost::shared_ptr<T3> mfc = boost::make_shared<T3>( key , frequency, sessionPtr_, key );
    mfc->registerCallback( message_actions::PUBLISH, boost::bind(&T1::publish, mfp, _1) );
    mfc->registerCallback( message_actions::RECORD, boost::bind(&T2::write, mfr, _1) );
    mfc->registerCallback( message_actions::LOG, boost::bind(&T2::bufferize, mfr, _1) );
    registerConverter( mfc, mfp, mfr );
  }

  void rosLoop();

  boost::scoped_ptr<ros::NodeHandle> nhPtr_;
  boost::mutex mutex_reinit_;
  boost::mutex mutex_conv_queue_;
  boost::mutex mutex_record_;

  std::vector< converter::Converter > converters_;
  std::map< std::string, publisher::Publisher > pub_map_;
  std::map< std::string, recorder::Recorder > rec_map_;
  std::map< std::string, event::Event > event_map_;
  typedef std::map< std::string, publisher::Publisher>::const_iterator PubConstIter;
  typedef std::map< std::string, publisher::Publisher>::iterator PubIter;
  typedef std::map< std::string, recorder::Recorder>::const_iterator RecConstIter;
  typedef std::map< std::string, recorder::Recorder>::iterator RecIter;
  typedef std::map< std::string, event::Event>::const_iterator EventConstIter;
  typedef std::map< std::string, event::Event>::iterator EventIter;

  std::vector< subscriber::Subscriber > subscribers_;
  std::vector< service::Service > services_;

  float buffer_duration_;

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

} // naoqi

#endif
