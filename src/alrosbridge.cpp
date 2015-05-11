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

/*
 * PUBLIC INTERFACE
 */
#include <alrosbridge/alrosbridge.hpp>
#include <alrosbridge/message_actions.h>

/*
 * CONVERTERS
 */
#include "converters/audio.hpp"
#include "converters/camera.hpp"
#include "converters/diagnostics.hpp"
#include "converters/imu.hpp"
#include "converters/info.hpp"
#include "converters/joint_state.hpp"
#include "converters/laser.hpp"
#include "converters/memory_list.hpp"
#include "converters/sonar.hpp"
#include "converters/memory/bool.hpp"
#include "converters/memory/float.hpp"
#include "converters/memory/int.hpp"
#include "converters/memory/string.hpp"
#include "converters/log.hpp"

/*
 * TOOLS
 */
#include "tools/robot_description.hpp"
#include "tools/alvisiondefinitions.h" // for kTop...

/*
 * RECORDERS
 */
#include "recorder/basic.hpp"
#include "recorder/basic_event.hpp"
#include "recorder/camera.hpp"
#include "recorder/diagnostics.hpp"
#include "recorder/joint_state.hpp"
#include "recorder/sonar.hpp"

/*
 * EVENTS
 */
#include "event/basic.hpp"
#include "event/audio.hpp"

/*
 * STATIC FUNCTIONS INCLUDE
 */
#include "ros_env.hpp"
#include "helpers.hpp"

/*
 * ROS
 */
#include <tf2_ros/buffer.h>

/*
 * BOOST
 */
#include <boost/foreach.hpp>
#include <boost/property_tree/json_parser.hpp>
#define for_each BOOST_FOREACH

#define DEBUG 0

namespace alros
{

Bridge::Bridge( qi::SessionPtr& session )
  : sessionPtr_( session ),
  freq_(15),
  record_enabled_(false),
  dump_enabled_(false),
  keep_looping(true),
  recorder_(boost::make_shared<recorder::GlobalRecorder>(::alros::ros_env::getPrefix())),
  buffer_duration_(helpers::bufferDefaultDuration)
{
}

Bridge::~Bridge()
{
  std::cout << "ALRosBridge is shutting down.." << std::endl;
}

void Bridge::init()
{
  ros::Time::init(); // can call this many times
  loadBootConfig();
  registerDefaultConverter();
  startRosLoop();
}

void Bridge::loadBootConfig()
{
  std::string file_path = alros::helpers::getBootConfigFile();
  std::cout << "load boot config from " << file_path << std::endl;
  boost::property_tree::read_json( file_path, boot_config_ );
}

void Bridge::stopService() {
  stopRosLoop();
  converters_.clear();
  event_map_.clear();
}


void Bridge::rosLoop()
{
  static std::vector<message_actions::MessageAction> actions;

//  ros::Time::init();
  while( keep_looping )
  {
    // clear the callback triggers
    actions.clear();
    {
      boost::mutex::scoped_lock lock( mutex_conv_queue_ );
      if (!conv_queue_.empty())
      {
        // Wait for the next Publisher to be ready
        size_t conv_index = conv_queue_.top().conv_index_;
        converter::Converter& conv = converters_[conv_index];
        ros::Time schedule = conv_queue_.top().schedule_;

        // check the recording condition
        // 1. recording enabled
        // 2. has to be registered
        // 3. has to be subscribed (configured to be recorded)
        RecConstIter rec_it = rec_map_.find( conv.name() );
        {
          boost::mutex::scoped_lock lock_record( mutex_record_, boost::try_to_lock );
          if ( lock_record && record_enabled_ && rec_it != rec_map_.end() && rec_it->second.isSubscribed() )
          {
            actions.push_back(message_actions::RECORD);
          }
        }

        // bufferize data in recorder
        if ( !dump_enabled_ && rec_it != rec_map_.end() && conv.frequency() != 0)
        {
          actions.push_back(message_actions::LOG);
        }

        if (actions.size() >0)
        {
          conv.callAll( actions );
        }

        //qi::Duration qi_now = qi::WallClock::now().time_since_epoch();
        //ros::Time ros_now = ros::Time( 0, qi_now.count() );
        //ros::Duration d(schedule - ros_now );
        ros::Duration d( schedule - ros::Time::now() );
        if ( d > ros::Duration(0))
        {
          d.sleep();
        }

#if DEBUG
        // check the publishing condition
        // publishing enabled
        // has to be registered
        // has to be subscribed
        ros::Time before = ros::Time::now();
        ros::Time after = ros::Time::now();
        std::cerr << "round trip last " << after-before << std::endl;
#endif
        // Schedule for a future time or not
        conv_queue_.pop();
        if ( conv.frequency() != 0 )
        {
          conv_queue_.push(ScheduledConverter(schedule + ros::Duration(1.0f / conv.frequency()), conv_index));
        }

      }
      else // conv_queue is empty.
      {
        // sleep one second
        ros::Duration(1).sleep();
      }
    } // mutex scope

    /*if ( publish_enabled_ )
    {
      ros::spinOnce();
    }*/
  } // while loop
}

std::string Bridge::minidump(const std::string& prefix)
{
  // IF A ROSBAG WAS OPENED, FIRST CLOSE IT
  if (record_enabled_)
  {
    stopRecording();
  }

  // STOP BUFFERIZING
  dump_enabled_ = true;
  for(EventIter iterator = event_map_.begin(); iterator != event_map_.end(); iterator++)
  {
    iterator->second.isDumping(true);
  }
  ros::Time time = ros::Time::now();

  // START A NEW ROSBAG
  boost::mutex::scoped_lock lock_record( mutex_record_ );
  recorder_->startRecord(prefix);

  // WRITE ALL BUFFER INTO THE ROSBAG
  for(EventIter iterator = event_map_.begin(); iterator != event_map_.end(); iterator++)
  {
    iterator->second.writeDump(time);
  }
  for(RecIter iterator = rec_map_.begin(); iterator != rec_map_.end(); iterator++)
  {
    iterator->second.writeDump(time);
  }

  // RESTART BUFFERIZING
  dump_enabled_ = false;
  for(EventIter iterator = event_map_.begin(); iterator != event_map_.end(); iterator++)
  {
    iterator->second.isDumping(false);
  }
  return recorder_->stopRecord(::alros::ros_env::getROSIP("eth0"));
}

std::string Bridge::minidumpConverters(const std::string& prefix, const std::vector<std::string>& names)
{
  // IF A ROSBAG WAS OPENED, FIRST CLOSE IT
  if (record_enabled_)
  {
    stopRecording();
  }

  // STOP BUFFERIZING
  dump_enabled_ = true;
  for(EventIter iterator = event_map_.begin(); iterator != event_map_.end(); iterator++)
  {
    iterator->second.isDumping(true);
  }
  ros::Time time = ros::Time::now();

  // WRITE CHOOSEN BUFFER INTO THE ROSBAG
  boost::mutex::scoped_lock lock_record( mutex_record_ );

  bool is_started = false;
  for_each( const std::string& name, names)
  {
    RecIter it = rec_map_.find(name);
    if ( it != rec_map_.end() )
    {
      if ( !is_started )
      {
        recorder_->startRecord(prefix);
        is_started = true;
      }
      it->second.writeDump(time);
    }
    else
    {
      EventIter it_event = event_map_.find(name);
      if ( it_event != event_map_.end() )
      {
        if ( !is_started )
        {
          recorder_->startRecord(prefix);
          is_started = true;
        }
        it_event->second.writeDump(time);
      }
    }
  }
  // RESTART BUFFERIZING
  dump_enabled_ = false;
  for(EventIter iterator = event_map_.begin(); iterator != event_map_.end(); iterator++)
  {
    iterator->second.isDumping(false);
  }
  if ( is_started )
  {
    return recorder_->stopRecord(::alros::ros_env::getROSIP("eth0"));
  }
  else
  {
    std::cout << BOLDRED << "Could not find any topic in recorders" << RESETCOLOR << std::endl
      << BOLDYELLOW << "To get the list of all available converter's name, please run:" << RESETCOLOR << std::endl
      << GREEN << "\t$ qicli call BridgeService.getAvailableConverters" << RESETCOLOR << std::endl;
    return "Could not find any topic in converters. To get the list of all available converter's name, please run: $ qicli call BridgeService.getAvailableConverters";
  }
}

void Bridge::setBufferDuration(float duration)
{
  for(RecIter iterator = rec_map_.begin(); iterator != rec_map_.end(); iterator++)
  {
    iterator->second.setBufferDuration(duration);
  }
  for(EventIter iterator = event_map_.begin(); iterator != event_map_.end(); iterator++)
  {
    iterator->second.setBufferDuration(duration);
  }
  buffer_duration_ = duration;
}

float Bridge::getBufferDuration()
{
  return buffer_duration_;
}

void Bridge::registerConverter( converter::Converter& conv )
{
  boost::mutex::scoped_lock lock( mutex_conv_queue_ );
  int conv_index = converters_.size();
  converters_.push_back( conv );
  conv.reset();
  conv_queue_.push(ScheduledConverter(ros::Time::now(), conv_index));
}

void Bridge::registerRecorder( const std::string& conv_name, recorder::Recorder& rec, float frequency)
{
  // Concept classes don't have any default constructors needed by operator[]
  // Cannot use this operator here. So we use insert
  rec.reset(recorder_, frequency);
  rec_map_.insert( std::map<std::string, recorder::Recorder>::value_type(conv_name, rec) );
}

void Bridge::insertEventConverter(const std::string& key, event::Event event)
{
  //event.reset(*nhPtr_, recorder_);
  event.resetRecorder(recorder_);
  event_map_.insert( std::map<std::string, event::Event>::value_type(key, event) );
}

void Bridge::registerConverter( converter::Converter conv, recorder::Recorder rec )
{
  registerConverter( conv );
  registerRecorder(  conv.name(), rec, conv.frequency());
}

bool Bridge::registerMemoryConverter( const std::string& key, float frequency, const dataType::DataType& type ) {
  dataType::DataType data_type;
  qi::AnyValue value;
  try {
    qi::AnyObject p_memory = sessionPtr_->service("ALMemory");
    value = p_memory.call<qi::AnyValue>("getData", key);
  } catch (const std::exception& e) {
    std::cout << BOLDRED << "Could not get data in memory for the key: "
              << BOLDCYAN << key << RESETCOLOR << std::endl;
    return false;
  }

  if (type==dataType::None) {
    try {
      data_type = helpers::getDataType(value);
    } catch (const std::exception& e) {
      std::cout << BOLDRED << "Could not get a valid data type to register memory converter "
                << BOLDCYAN << key << RESETCOLOR << std::endl
                << BOLDRED << "You can enter it yourself, available types are:" << std::endl
                << "\t > 0 - None" << std::endl
                << "\t > 1 - Float" << std::endl
                << "\t > 2 - Int" << std::endl
                << "\t > 3 - String" << std::endl
                << "\t > 4 - Bool" << RESETCOLOR << std::endl;
      return false;
    }
  }
  else {
    data_type = type;
  }

  switch (data_type) {
  case 0:
    return false;
    break;
  case 1:
    _registerMemoryConverter<converter::MemoryFloatConverter, recorder::BasicRecorder<naoqi_bridge_msgs::FloatStamped> >(key,frequency);
    break;
  case 2:
    _registerMemoryConverter<converter::MemoryIntConverter, recorder::BasicRecorder<naoqi_bridge_msgs::IntStamped> >(key,frequency);
    break;
  case 3:
    _registerMemoryConverter<converter::MemoryStringConverter, recorder::BasicRecorder<naoqi_bridge_msgs::StringStamped> >(key,frequency);
    break;
  case 4:
    _registerMemoryConverter<converter::MemoryBoolConverter, recorder::BasicRecorder<naoqi_bridge_msgs::BoolStamped> >(key,frequency);
    break;
  default:
    {
      std::cout << BOLDRED << "Wrong data type. Available type are: " << std::endl
                   << "\t > 0 - None" << std::endl
                   << "\t > 1 - Float" << std::endl
                   << "\t > 2 - Int" << std::endl
                   << "\t > 3 - String" << std::endl
                   << "\t > 4 - Bool" << RESETCOLOR << std::endl;
      return false;
      break;
    }
  }
  return true;
}

void Bridge::registerDefaultConverter()
{
  // init global tf2 buffer
  tf2_buffer_.reset<tf2_ros::Buffer>( new tf2_ros::Buffer() );
  tf2_buffer_->setUsingDedicatedThread(true);

  alros::Robot robot_type;

  // replace this with proper configuration struct
  bool info_enabled                   = boot_config_.get( "converters.info.enabled", true);
  size_t info_frequency               = boot_config_.get( "converters.info.frequency", 1);

  bool audio_enabled                  = boot_config_.get( "converters.audio.enabled", true);
  size_t audio_frequency              = boot_config_.get( "converters.audio.frequency", 1);

  bool logs_enabled                   = boot_config_.get( "converters.logs.enabled", true);
  size_t logs_frequency               = boot_config_.get( "converters.logs.frequency", 10);

  bool diag_enabled                   = boot_config_.get( "converters.diag.enabled", true);
  size_t diag_frequency               = boot_config_.get( "converters.diag.frequency", 10);

  bool imu_torso_enabled              = boot_config_.get( "converters.imu_torso.enabled", true);
  size_t imu_torso_frequency          = boot_config_.get( "converters.imu_torso.frequency", 10);

  bool imu_base_enabled               = boot_config_.get( "converters.imu_base.enabled", true);
  size_t imu_base_frequency           = boot_config_.get( "converters.imu_base.frequency", 10);

  bool camera_front_enabled           = boot_config_.get( "converters.front_camera.enabled", true);
  size_t camera_front_resolution      = boot_config_.get( "converters.front_camera.resolution", 2); // VGA
  size_t camera_front_fps             = boot_config_.get( "converters.front_camera.fps", 15);
  size_t camera_front_recorder_fps    = boot_config_.get( "converters.front_camera.recorder_fps", 5);

  bool camera_depth_enabled           = boot_config_.get( "converters.depth_camera.enabled", true);
  size_t camera_depth_resolution      = boot_config_.get( "converters.depth_camera.resolution", 0); // QQVGA
  size_t camera_depth_fps             = boot_config_.get( "converters.depth_camera.fps", 15);
  size_t camera_depth_recorder_fps    = boot_config_.get( "converters.depth_camera.recorder_fps", 5);

  bool camera_ir_enabled              = boot_config_.get( "converters.ir_camera.enabled", true);
  size_t camera_ir_resolution         = boot_config_.get( "converters.ir_camera.resolution", 0); // QQVGA
  size_t camera_ir_fps                = boot_config_.get( "converters.ir_camera.fps", 15);
  size_t camera_ir_recorder_fps       = boot_config_.get( "converters.ir_camera.recorder_fps", 5);

  bool joint_states_enabled           = boot_config_.get( "converters.joint_states.enabled", true);
  size_t joint_states_frequency       = boot_config_.get( "converters.joint_states.frequency", 15);

  bool laser_enabled                  = boot_config_.get( "converters.laser.enabled", true);
  size_t laser_frequency              = boot_config_.get( "converters.laser.frequency", 10);

  bool sonar_enabled                  = boot_config_.get( "converters.sonar.enabled", true);
  size_t sonar_frequency              = boot_config_.get( "converters.sonar.frequency", 10);

  /*
   * The info converter will be called once after it was added to the priority queue. Once it is its turn to be called, its
   * callAll method will be triggered (because InfoPublisher is considered to always have subscribers, isSubscribed always
   * return true).
   * A message is therefore published through InfoPublisher, even if there is nobody to receive it.
   * Then, InfoConverter will never be called again, because of its 0Hz frequency. But if a new user subscribes to the "info"
   * topic, he/she will receive the information published before, as the publisher is latched.
   */
  /** Info publisher **/
  boost::shared_ptr<converter::InfoConverter> inc = boost::make_shared<converter::InfoConverter>( "info", 0, sessionPtr_ );
  robot_type = inc->robot();

  if ( info_enabled )
  {
    boost::shared_ptr<recorder::BasicRecorder<naoqi_bridge_msgs::StringStamped> > inr = boost::make_shared<recorder::BasicRecorder<naoqi_bridge_msgs::StringStamped> >( "info" );
    inc->registerCallback( message_actions::RECORD, boost::bind(&recorder::BasicRecorder<naoqi_bridge_msgs::StringStamped>::write, inr, _1) );
    inc->registerCallback( message_actions::LOG, boost::bind(&recorder::BasicRecorder<naoqi_bridge_msgs::StringStamped>::bufferize, inr, _1) );
    registerConverter( inc, inr );
  }

  /** LOGS */
  if ( logs_enabled )
  {
    boost::shared_ptr<converter::LogConverter> lc = boost::make_shared<converter::LogConverter>( "log", logs_frequency, sessionPtr_);
    boost::shared_ptr<recorder::BasicRecorder<rosgraph_msgs::Log> > lr = boost::make_shared<recorder::BasicRecorder<rosgraph_msgs::Log> >( "log" );
    lc->registerCallback( message_actions::RECORD, boost::bind(&recorder::BasicRecorder<rosgraph_msgs::Log>::write, lr, _1) );
    lc->registerCallback( message_actions::LOG, boost::bind(&recorder::BasicRecorder<rosgraph_msgs::Log>::bufferize, lr, _1) );
    registerConverter( lc, lr );
  }

  /** DIAGNOSTICS */
  if ( diag_enabled )
  {
    boost::shared_ptr<converter::DiagnosticsConverter> dc = boost::make_shared<converter::DiagnosticsConverter>( "diag", diag_frequency, sessionPtr_);
    boost::shared_ptr<recorder::DiagnosticsRecorder>   dr = boost::make_shared<recorder::DiagnosticsRecorder>( "/diagnostics_agg" );
    dc->registerCallback( message_actions::RECORD, boost::bind(&recorder::DiagnosticsRecorder::write, dr, _1) );
    dc->registerCallback( message_actions::LOG, boost::bind(&recorder::DiagnosticsRecorder::bufferize, dr, _1) );
    registerConverter( dc, dr );
  }

  /** IMU TORSO **/
  if ( imu_torso_enabled )
  {
    boost::shared_ptr<recorder::BasicRecorder<sensor_msgs::Imu> > imutr = boost::make_shared<recorder::BasicRecorder<sensor_msgs::Imu> >( "imu_torso" );
    boost::shared_ptr<converter::ImuConverter> imutc = boost::make_shared<converter::ImuConverter>( "imu_torso", converter::IMU::TORSO, imu_torso_frequency, sessionPtr_);
    imutc->registerCallback( message_actions::RECORD, boost::bind(&recorder::BasicRecorder<sensor_msgs::Imu>::write, imutr, _1) );
    imutc->registerCallback( message_actions::LOG, boost::bind(&recorder::BasicRecorder<sensor_msgs::Imu>::bufferize, imutr, _1) );
    registerConverter( imutc, imutr );
  }

  if(robot_type == alros::PEPPER)
  {
    /** IMU BASE **/
    if ( imu_base_enabled )
    {
      boost::shared_ptr<recorder::BasicRecorder<sensor_msgs::Imu> > imubr = boost::make_shared<recorder::BasicRecorder<sensor_msgs::Imu> >( "imu_base" );
      boost::shared_ptr<converter::ImuConverter> imubc = boost::make_shared<converter::ImuConverter>( "imu_base", converter::IMU::BASE, imu_base_frequency, sessionPtr_);
      imubc->registerCallback( message_actions::RECORD, boost::bind(&recorder::BasicRecorder<sensor_msgs::Imu>::write, imubr, _1) );
      imubc->registerCallback( message_actions::LOG, boost::bind(&recorder::BasicRecorder<sensor_msgs::Imu>::bufferize, imubr, _1) );
      registerConverter( imubc, imubr );
    }
  } // endif PEPPER

  /** Front Camera */
  if ( camera_front_enabled )
  {
    boost::shared_ptr<recorder::CameraRecorder> fcr = boost::make_shared<recorder::CameraRecorder>( "camera/front", camera_front_recorder_fps );
    boost::shared_ptr<converter::CameraConverter> fcc = boost::make_shared<converter::CameraConverter>( "front_camera", camera_front_fps, sessionPtr_, AL::kTopCamera, camera_front_resolution );
    fcc->registerCallback( message_actions::RECORD, boost::bind(&recorder::CameraRecorder::write, fcr, _1, _2) );
    fcc->registerCallback( message_actions::LOG, boost::bind(&recorder::CameraRecorder::bufferize, fcr, _1, _2) );
    registerConverter( fcc, fcr );
  }

  if(robot_type == alros::PEPPER)
  {
    /** Depth Camera */
    if ( camera_depth_enabled )
    {
      boost::shared_ptr<recorder::CameraRecorder> dcr = boost::make_shared<recorder::CameraRecorder>( "camera/depth", camera_depth_recorder_fps );
      boost::shared_ptr<converter::CameraConverter> dcc = boost::make_shared<converter::CameraConverter>( "depth_camera", camera_depth_fps, sessionPtr_, AL::kDepthCamera, camera_depth_resolution );
      dcc->registerCallback( message_actions::RECORD, boost::bind(&recorder::CameraRecorder::write, dcr, _1, _2) );
      dcc->registerCallback( message_actions::LOG, boost::bind(&recorder::CameraRecorder::bufferize, dcr, _1, _2) );
      registerConverter( dcc, dcr );
    }

    /** Infrared Camera */
    if ( camera_ir_enabled )
    {
      boost::shared_ptr<recorder::CameraRecorder> icr = boost::make_shared<recorder::CameraRecorder>( "camera/ir", camera_ir_recorder_fps );
      boost::shared_ptr<converter::CameraConverter> icc = boost::make_shared<converter::CameraConverter>( "infrared_camera", camera_ir_fps, sessionPtr_, AL::kInfraredCamera, camera_ir_resolution);
      icc->registerCallback( message_actions::RECORD, boost::bind(&recorder::CameraRecorder::write, icr, _1, _2) );
      icc->registerCallback( message_actions::LOG, boost::bind(&recorder::CameraRecorder::bufferize, icr, _1, _2) );
      registerConverter( icc, icr );
    }
  } // endif PEPPER

  /** Joint States */
  if ( joint_states_enabled )
  {
    boost::shared_ptr<recorder::JointStateRecorder> jsr = boost::make_shared<recorder::JointStateRecorder>( "/joint_states" );
    boost::shared_ptr<converter::JointStateConverter> jsc = boost::make_shared<converter::JointStateConverter>( "joint_states", joint_states_frequency, tf2_buffer_, sessionPtr_ );
    jsc->registerCallback( message_actions::RECORD, boost::bind(&recorder::JointStateRecorder::write, jsr, _1, _2) );
    jsc->registerCallback( message_actions::LOG, boost::bind(&recorder::JointStateRecorder::bufferize, jsr, _1, _2) );
    registerConverter( jsc, jsr );
  }

  if(robot_type == alros::PEPPER){
    /** Laser */
    if ( laser_enabled )
    {
      boost::shared_ptr<recorder::BasicRecorder<sensor_msgs::LaserScan> > lr = boost::make_shared<recorder::BasicRecorder<sensor_msgs::LaserScan> >( "laser" );
      boost::shared_ptr<converter::LaserConverter> lc = boost::make_shared<converter::LaserConverter>( "laser", laser_frequency, sessionPtr_ );
      lc->registerCallback( message_actions::RECORD, boost::bind(&recorder::BasicRecorder<sensor_msgs::LaserScan>::write, lr, _1) );
      lc->registerCallback( message_actions::LOG, boost::bind(&recorder::BasicRecorder<sensor_msgs::LaserScan>::bufferize, lr, _1) );
      registerConverter( lc, lr );
    }
  }

  /** Sonar */
  if ( sonar_enabled )
  {
    std::vector<std::string> sonar_topics;
    if (robot_type == alros::PEPPER)
    {
      sonar_topics.push_back("sonar/front");
      sonar_topics.push_back("sonar/back");
    }
    else
    {
      sonar_topics.push_back("sonar/left");
      sonar_topics.push_back("sonar/right");
    }
    boost::shared_ptr<recorder::SonarRecorder> usr = boost::make_shared<recorder::SonarRecorder>( sonar_topics );
    boost::shared_ptr<converter::SonarConverter> usc = boost::make_shared<converter::SonarConverter>( "sonar", sonar_frequency, sessionPtr_ );
    usc->registerCallback( message_actions::RECORD, boost::bind(&recorder::SonarRecorder::write, usr, _1) );
    usc->registerCallback( message_actions::LOG, boost::bind(&recorder::SonarRecorder::bufferize, usr, _1) );
    registerConverter( usc, usr );
  }

  {
    /** Audio */
    boost::shared_ptr<AudioEventRegister> event_register =
        boost::make_shared<AudioEventRegister>( "audio", 0, sessionPtr_ );
    insertEventConverter("audio", event_register);
    if (keep_looping) {
      event_map_.find("audio")->second.startProcess();
    }
  }
}

std::vector<std::string> Bridge::getAvailableConverters()
{
  std::vector<std::string> conv_list;
  for_each( const converter::Converter& conv, converters_ )
  {
    conv_list.push_back(conv.name());
  }
  for(EventConstIter iterator = event_map_.begin(); iterator != event_map_.end(); iterator++)
  {
    conv_list.push_back( iterator->first );
  }

  return conv_list;
}

/*
* EXPOSED FUNCTIONS
*/

void Bridge::startRecording()
{
  boost::mutex::scoped_lock lock_record( mutex_record_ );
  recorder_->startRecord();
  for_each( converter::Converter& conv, converters_ )
  {
    RecIter it = rec_map_.find(conv.name());
    if ( it != rec_map_.end() )
    {
      it->second.subscribe(true);
      std::cout << HIGHGREEN << "Topic "
                << BOLDCYAN << conv.name() << RESETCOLOR
                << HIGHGREEN << " is subscribed for recording" << RESETCOLOR << std::endl;
    }
  }
  for(EventIter iterator = event_map_.begin(); iterator != event_map_.end(); iterator++)
  {
    iterator->second.isRecording(true);
    std::cout << HIGHGREEN << "Topic "
              << BOLDCYAN << iterator->first << RESETCOLOR
              << HIGHGREEN << " is subscribed for recording" << RESETCOLOR << std::endl;
  }
  record_enabled_ = true;
}

void Bridge::startRecordingConverters(const std::vector<std::string>& names)
{
  boost::mutex::scoped_lock lock_record( mutex_record_ );

  bool is_started = false;
  for_each( const std::string& name, names)
  {
    RecIter it_rec = rec_map_.find(name);
    EventIter it_ev = event_map_.find(name);
    if ( it_rec != rec_map_.end() )
    {
      if ( !is_started )
      {
        recorder_->startRecord();
        is_started = true;
      }
      it_rec->second.subscribe(true);
      std::cout << HIGHGREEN << "Topic "
        << BOLDCYAN << name << RESETCOLOR
        << HIGHGREEN << " is subscribed for recording" << RESETCOLOR << std::endl;
    }
    else if ( it_ev != event_map_.end() )
    {
      if ( !is_started )
      {
        recorder_->startRecord();
        is_started = true;
      }
      it_ev->second.isRecording(true);
      std::cout << HIGHGREEN << "Topic "
        << BOLDCYAN << name << RESETCOLOR
        << HIGHGREEN << " is subscribed for recording" << RESETCOLOR << std::endl;
    }
    else
    {
      std::cout << BOLDRED << "Could not find topic "
        << BOLDCYAN << name
        << BOLDRED << " in recorders" << RESETCOLOR << std::endl
        << BOLDYELLOW << "To get the list of all available converter's name, please run:" << RESETCOLOR << std::endl
        << GREEN << "\t$ qicli call BridgeService.getAvailableConverters" << RESETCOLOR << std::endl;
    }
  }
  if ( is_started )
  {
    record_enabled_ = true;
  }
  else
  {
    std::cout << BOLDRED << "Could not find any topic in recorders" << RESETCOLOR << std::endl
      << BOLDYELLOW << "To get the list of all available converter's name, please run:" << RESETCOLOR << std::endl
      << GREEN << "\t$ qicli call BridgeService.getAvailableConverters" << RESETCOLOR << std::endl;
  }
}

std::string Bridge::stopRecording()
{
  boost::mutex::scoped_lock lock_record( mutex_record_ );
  record_enabled_ = false;
  for_each( converter::Converter& conv, converters_ )
  {
    RecIter it = rec_map_.find(conv.name());
    if ( it != rec_map_.end() )
    {
      it->second.subscribe(false);
    }
  }
  for(EventIter iterator = event_map_.begin(); iterator != event_map_.end(); iterator++)
  {
    iterator->second.isRecording(false);
  }
  return recorder_->stopRecord(::alros::ros_env::getROSIP("eth0"));
}

void Bridge::startRosLoop()
{
  if (publisherThread_.get_id() ==  boost::thread::id())
    publisherThread_ = boost::thread( &Bridge::rosLoop, this );
  for(EventIter iterator = event_map_.begin(); iterator != event_map_.end(); iterator++)
  {
    iterator->second.startProcess();
  }
  // Create the publishing thread if needed
  keep_looping = true;
}

void Bridge::stopRosLoop()
{
  keep_looping = false;
  if (publisherThread_.get_id() !=  boost::thread::id())
    publisherThread_.join();
  for(EventIter iterator = event_map_.begin(); iterator != event_map_.end(); iterator++)
  {
    iterator->second.stopProcess();
  }
}

void Bridge::parseJsonFile(std::string filepath, boost::property_tree::ptree &pt){
  // Open json file and parse it
  std::ifstream json_file;
  json_file.open(filepath.c_str(), std::ios_base::in);

  boost::property_tree::json_parser::read_json(json_file, pt);
  json_file.close();
}

void Bridge::addMemoryConverters(std::string filepath){
  // Open the file filepath and parse it
  boost::property_tree::ptree pt;
  parseJsonFile(filepath, pt);


  // Get the frequency requested (default to 10 Hz)
  float frequency = 10.0f;
  try{
    frequency = pt.get<float>("frequency");
  }
  catch(const boost::property_tree::ptree_bad_data& e){
    std::cout << "\"frequency\" could not be interpreted as float: " <<  e.what() << std::endl;
    std::cout << "Default to 10 Hz" << std::endl;
  }
  catch(const boost::property_tree::ptree_bad_path& e){
    std::cout << "\"frequency\" was not found: " <<  e.what() << std::endl;
    std::cout << "Default to 10 Hz" << std::endl;
  }

  // Get the topic name requested
  std::string topic;
  try{
    topic = pt.get<std::string>("topic");
  }
  catch(const boost::property_tree::ptree_error& e){
    std::cout << "\"topic\" could not be retrieved: " <<  e.what() << std::endl
              << "Cannot add new converters" << std::endl;
    return;
  }

  std::vector<std::string> list;
  try{
    BOOST_FOREACH(boost::property_tree::ptree::value_type &v, pt.get_child("memKeys"))
    {
      std::string topic = v.second.get_value<std::string>();
      list.push_back(topic);
    }
  }
  catch(const boost::property_tree::ptree_error& e){
    std::cout << "A problem occured during the reading of the mem keys list: " << e.what() << std::endl
              << "Cannot add new converters" << std::endl;
    return;
  }

  if(list.empty()){
    std::cout << "The list of keys to add is empty. " << std::endl;
    return;
  }

  // Create converter, publisher and recorder
  boost::shared_ptr<recorder::BasicRecorder<naoqi_bridge_msgs::MemoryList> > mlr = boost::make_shared<recorder::BasicRecorder<naoqi_bridge_msgs::MemoryList> >( topic );
  boost::shared_ptr<converter::MemoryListConverter> mlc = boost::make_shared<converter::MemoryListConverter>(list, topic, frequency, sessionPtr_ );
  mlc->registerCallback( message_actions::RECORD, boost::bind(&recorder::BasicRecorder<naoqi_bridge_msgs::MemoryList>::write, mlr, _1) );
  mlc->registerCallback( message_actions::LOG, boost::bind(&recorder::BasicRecorder<naoqi_bridge_msgs::MemoryList>::bufferize, mlr, _1) );

  registerConverter( mlc, mlr );
}

bool Bridge::registerEventConverter(const std::string& key, const dataType::DataType& type)
{
  dataType::DataType data_type;
  qi::AnyValue value;
  try {
    qi::AnyObject p_memory = sessionPtr_->service("ALMemory");
    value = p_memory.call<qi::AnyValue>("getData", key);
  } catch (const std::exception& e) {
    std::cout << BOLDRED << "Could not get data in memory for the key: "
              << BOLDCYAN << key << RESETCOLOR << std::endl;
    return false;
  }

  if (type==dataType::None) {
    try {
      data_type = helpers::getDataType(value);
    } catch (const std::exception& e) {
      std::cout << BOLDRED << "Could not get a valid data type to register memory converter "
                << BOLDCYAN << key << RESETCOLOR << std::endl
                << BOLDRED << "You can enter it yourself, available types are:" << std::endl
                << "\t > 0 - None" << std::endl
                << "\t > 1 - Float" << std::endl
                << "\t > 2 - Int" << std::endl
                << "\t > 3 - String" << std::endl
                << "\t > 4 - Bool" << RESETCOLOR << std::endl;
      return false;
    }
  }
  else {
    data_type = type;
  }

  switch (data_type) {
  case 0:
    return false;
    break;
  case 1:
    {
      boost::shared_ptr<EventRegister<converter::MemoryFloatConverter,recorder::BasicEventRecorder<naoqi_bridge_msgs::FloatStamped> > > event_register =
          boost::make_shared<EventRegister<converter::MemoryFloatConverter,recorder::BasicEventRecorder<naoqi_bridge_msgs::FloatStamped> > >( key, sessionPtr_ );
      insertEventConverter(key, event_register);
      break;
    }
  case 2:
    {
      boost::shared_ptr<EventRegister<converter::MemoryIntConverter,recorder::BasicEventRecorder<naoqi_bridge_msgs::IntStamped> > > event_register =
          boost::make_shared<EventRegister<converter::MemoryIntConverter,recorder::BasicEventRecorder<naoqi_bridge_msgs::IntStamped> > >( key, sessionPtr_ );
      insertEventConverter(key, event_register);
      break;
    }
  case 3:
    {
      boost::shared_ptr<EventRegister<converter::MemoryStringConverter,recorder::BasicEventRecorder<naoqi_bridge_msgs::StringStamped> > > event_register =
          boost::make_shared<EventRegister<converter::MemoryStringConverter,recorder::BasicEventRecorder<naoqi_bridge_msgs::StringStamped> > >( key, sessionPtr_ );
      insertEventConverter(key, event_register);
      break;
    }
  case 4:
    {
      boost::shared_ptr<EventRegister<converter::MemoryBoolConverter,recorder::BasicEventRecorder<naoqi_bridge_msgs::BoolStamped> > > event_register =
          boost::make_shared<EventRegister<converter::MemoryBoolConverter,recorder::BasicEventRecorder<naoqi_bridge_msgs::BoolStamped> > >( key, sessionPtr_ );
      insertEventConverter(key, event_register);
      break;
    }
  default:
    {
      std::cout << BOLDRED << "Wrong data type. Available type are: " << std::endl
                   << "\t > 0 - None" << std::endl
                   << "\t > 1 - Float" << std::endl
                   << "\t > 2 - Int" << std::endl
                   << "\t > 3 - String" << std::endl
                   << "\t > 4 - Bool" << RESETCOLOR << std::endl;
      return false;
    }
  }

  if (keep_looping) {
    event_map_.find(key)->second.startProcess();
  }

  return true;
}

QI_REGISTER_OBJECT( Bridge,
                    _whoIsYourDaddy,
                    minidump,
                    minidumpConverters,
                    setBufferDuration,
                    getBufferDuration,
                    getAvailableConverters,
                    addMemoryConverters,
                    registerMemoryConverter,
                    registerEventConverter,
                    startRecording,
                    startRecordingConverters,
                    stopRecording );
} //alros
