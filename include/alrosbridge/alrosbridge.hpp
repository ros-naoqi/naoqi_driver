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
#include <alrosbridge/recorder/recorder.hpp>
#include <alrosbridge/event/event.hpp>
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
   * @brief prepare and register a recorder
   * @param conv_name the name of the converter related to the recorder
   * @param rec       the recorder to add
   */
  void registerRecorder(const std::string& conv_name, recorder::Recorder& rec, float frequency);

  /**
   * @brief register a converter with an associated publisher and recorder
   */
  void registerConverter(converter::Converter conv, recorder::Recorder rec );

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
  * @brief qicli call function to start recording all registered converter in a ROSbag
  */
  void _startRecording();

  /**
  * @brief qicli call function to start recording given topics in a ROSbag
  */
  void _startRecordingConverters(const std::vector<std::string>& names);

  /**
  * @brief qicli call function to stop recording all registered publisher in a ROSbag
  */
  std::string _stopRecording();

  /**
   * @brief qicli call function to add on-the-fly some memory keys extractors
   */
  void addMemoryConverters(std::string filepath);

  void parseJsonFile(std::string filepath, boost::property_tree::ptree& pt);

  void stopService();

  std::vector<std::string> getFilesList();
  std::string _whoIsYourDaddy()
  {
    return "ask surya";
  }

private:
  qi::SessionPtr sessionPtr_;
  bool record_enabled_;
  bool dump_enabled_;
  bool keep_looping;

  const size_t freq_;
  boost::thread publisherThread_;

  boost::shared_ptr<recorder::GlobalRecorder> recorder_;

  /* boot config */
  boost::property_tree::ptree boot_config_;
  void loadBootConfig();

  void registerDefaultConverter();
  void insertEventConverter(const std::string& key, event::Event event);

  template <typename T1, typename T2>
  void _registerMemoryConverter( const std::string& key, float frequency ) {
    boost::shared_ptr<T2> mfr = boost::make_shared<T2>( key );
    boost::shared_ptr<T1> mfc = boost::make_shared<T1>( key , frequency, sessionPtr_, key );
    mfc->registerCallback( message_actions::RECORD, boost::bind(&T2::write, mfr, _1) );
    mfc->registerCallback( message_actions::LOG, boost::bind(&T2::bufferize, mfr, _1) );
    registerConverter( mfc, mfr );
  }

  void rosLoop();

  boost::mutex mutex_reinit_;
  boost::mutex mutex_conv_queue_;
  boost::mutex mutex_record_;

  std::vector< converter::Converter > converters_;
  std::map< std::string, recorder::Recorder > rec_map_;
  std::map< std::string, event::Event > event_map_;
  typedef std::map< std::string, recorder::Recorder>::const_iterator RecConstIter;
  typedef std::map< std::string, recorder::Recorder>::iterator RecIter;
  typedef std::map< std::string, event::Event>::const_iterator EventConstIter;
  typedef std::map< std::string, event::Event>::iterator EventIter;

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

} // alros

#endif
