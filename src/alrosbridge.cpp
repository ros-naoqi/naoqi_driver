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
 * PUBLISHERS
 */
#include "publishers/basic.hpp"
#include "publishers/camera.hpp"
#include "publishers/info.hpp"
#include "publishers/joint_state.hpp"
#include "publishers/log.hpp"
#include "publishers/sonar.hpp"

/*
 * TOOLS
 */
#include "tools/robot_description.hpp"
#include "tools/alvisiondefinitions.h" // for kTop...

/*
 * SUBSCRIBERS
 */
#include "subscribers/teleop.hpp"
#include "subscribers/moveto.hpp"

/*
 * RECORDERS
 */
#include "recorder/basic.hpp"
#include "recorder/camera.hpp"
#include "recorder/diagnostics.hpp"
#include "recorder/joint_state.hpp"
#include "recorder/sonar.hpp"

#include "event.hpp"

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
  publish_enabled_(true),
  record_enabled_(false),
  keep_looping(true),
  recorder_(boost::make_shared<recorder::GlobalRecorder>(::alros::ros_env::getPrefix()))
{
}

Bridge::~Bridge()
{
  std::cout << "ALRosBridge is shutting down.." << std::endl;
  // destroy nodehandle?
  if(nhPtr_)
  {
    nhPtr_->shutdown();
    ros::shutdown();
  }
}

void Bridge::stopService() {
  stopRosLoop();
  converters_.clear();
  subscribers_.clear();
  event_map_.clear();
}


void Bridge::rosLoop()
{
  static std::vector<message_actions::MessageAction> actions;

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

        PubConstIter pub_it = pub_map_.find( conv.name() );
        if ( publish_enabled_ &&  pub_it != pub_map_.end() && pub_it->second.isSubscribed() )
        {
          actions.push_back(message_actions::PUBLISH);
        }

        // check the recording condition
        // recording enabled
        // has to be registered
        // has to be subscribed (configured to be recorded)
        RecConstIter rec_it = rec_map_.find( conv.name() );
        {
          boost::mutex::scoped_lock lock_record( mutex_record_ );
          if ( record_enabled_ && rec_it != rec_map_.end() && rec_it->second.isSubscribed() )
          {
            actions.push_back(message_actions::RECORD);
          }
        }

        // bufferize data in recorder
        if ( rec_it != rec_map_.end() && conv.frequency() != 0)
        {
          actions.push_back(message_actions::LOG);
        }

        if (actions.size() >0)
        {
          conv.callAll( actions );
        }

        ros::Duration d(schedule - ros::Time::now());
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
          conv_queue_.push(ScheduledConverter(schedule + ros::Duration(1.0f / conv.frequency()), conv_index));
      } else
        // sleep one second
        ros::Duration(1).sleep();
    }
    ros::spinOnce();
  }
}

void Bridge::minidump()
{
  // IF A ROSBAG WAS OPENED, FIRST CLOSE IT
  if (record_enabled_)
  {
    stopRecording();
  }

  // WRITE ALL BUFFER INTO THE ROSBAG
  boost::mutex::scoped_lock lock_record( mutex_record_ );
  recorder_->startRecord(); // MAYBE ADD NAME ARGUMENT
  // for each recorder, call write_dump function
  for(RecIter iterator = rec_map_.begin(); iterator != rec_map_.end(); iterator++)
  {
    iterator->second.writeDump();
  }
  recorder_->stopRecord(::alros::ros_env::getROSIP("eth0"));
}

void Bridge::setBufferDuration(float duration)
{
  // LOOP AGAINST ALL RECORDERS
  for(RecIter iterator = rec_map_.begin(); iterator != rec_map_.end(); iterator++)
  {
    iterator->second.setBufferDuration(duration);
  }
}

void Bridge::registerConverter( converter::Converter& conv )
{
  boost::mutex::scoped_lock lock( mutex_conv_queue_ );
  int conv_index = converters_.size();
  converters_.push_back( conv );
  conv.reset();
  conv_queue_.push(ScheduledConverter(ros::Time::now(), conv_index));
}

void Bridge::registerPublisher( const std::string& conv_name, publisher::Publisher& pub)
{
  pub.reset(*nhPtr_);
  // Concept classes don't have any default constructors needed by operator[]
  // Cannot use this operator here. So we use insert
  pub_map_.insert( std::map<std::string, publisher::Publisher>::value_type(conv_name, pub) );
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
  event.reset(*nhPtr_, recorder_);
  event_map_.insert( std::map<std::string, event::Event>::value_type(key, event) );
}

void Bridge::registerConverter( converter::Converter conv, publisher::Publisher pub, recorder::Recorder rec )
{
  registerConverter( conv );
  registerPublisher( conv.name(), pub);
  registerRecorder(  conv.name(), rec, conv.frequency());
}

void Bridge::registerPublisher( converter::Converter conv, publisher::Publisher pub )
{
  registerConverter( conv );
  registerPublisher(conv.name(), pub);
}

void Bridge::registerRecorder( converter::Converter conv, recorder::Recorder rec )
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
    _registerMemoryConverter<publisher::BasicPublisher<naoqi_bridge_msgs::FloatStamped>,recorder::BasicRecorder<naoqi_bridge_msgs::FloatStamped>,converter::MemoryFloatConverter>(key,frequency);
    break;
  case 2:
    _registerMemoryConverter<publisher::BasicPublisher<naoqi_bridge_msgs::IntStamped>,recorder::BasicRecorder<naoqi_bridge_msgs::IntStamped>,converter::MemoryIntConverter>(key,frequency);
    break;
  case 3:
    _registerMemoryConverter<publisher::BasicPublisher<naoqi_bridge_msgs::StringStamped>,recorder::BasicRecorder<naoqi_bridge_msgs::StringStamped>,converter::MemoryStringConverter>(key,frequency);
    break;
  case 4:
    _registerMemoryConverter<publisher::BasicPublisher<naoqi_bridge_msgs::BoolStamped>,recorder::BasicRecorder<naoqi_bridge_msgs::BoolStamped>,converter::MemoryBoolConverter>(key,frequency);
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

//  // Info should be at 0 (latched) but somehow that does not work ...
//  publisher::Publisher info = alros::publisher::InfoPublisher("info", "info", 0.1, sessionPtr_);
//  registerPublisher( info );

  alros::Robot robot_type;

  /** Info publisher **/
  boost::shared_ptr<converter::InfoConverter> inc = boost::make_shared<converter::InfoConverter>( "info", 0, sessionPtr_ );
  robot_type = inc->robot();

  /*
   * The info converter will be called once after it was added to the priority queue. Once it is its turn to be called, its
   * callAll method will be triggered (because InfoPublisher is considered to always have subscribers, isSubscribed always
   * return true).
   * A message is therefore published through InfoPublisher, even if there is nobody to receive it.
   * Then, InfoConverter will never be called again, because of its 0Hz frequency. But if a new user subscribes to the "info"
   * topic, he/she will receive the information published before, as the publisher is latched.
   */

  boost::shared_ptr<publisher::InfoPublisher> inp = boost::make_shared<publisher::InfoPublisher>( "info" , robot_type);
  boost::shared_ptr<recorder::BasicRecorder<naoqi_bridge_msgs::StringStamped> > inr = boost::make_shared<recorder::BasicRecorder<naoqi_bridge_msgs::StringStamped> >( "info" );
  inc->registerCallback( message_actions::PUBLISH, boost::bind(&publisher::InfoPublisher::publish, inp, _1) );
  inc->registerCallback( message_actions::RECORD, boost::bind(&recorder::BasicRecorder<naoqi_bridge_msgs::StringStamped>::write, inr, _1) );
  inc->registerCallback( message_actions::LOG, boost::bind(&recorder::BasicRecorder<naoqi_bridge_msgs::StringStamped>::bufferize, inr, _1) );
  registerConverter( inc, inp, inr );

  /** AUDIO **/
  boost::shared_ptr<converter::AudioConverter> ac = boost::make_shared<converter::AudioConverter>( "audio", 1, sessionPtr_);
  boost::shared_ptr<publisher::BasicPublisher<naoqi_msgs::AudioBuffer> > ap = boost::make_shared<publisher::BasicPublisher<naoqi_msgs::AudioBuffer> >( "audio" );
  ac->registerCallback( message_actions::PUBLISH, boost::bind(&publisher::BasicPublisher<naoqi_msgs::AudioBuffer>::publish, ap, _1));
  registerPublisher( ac, ap );

  /** LOGS */
  boost::shared_ptr<converter::LogConverter> lc = boost::make_shared<converter::LogConverter>( "log", 1, sessionPtr_);
  boost::shared_ptr<publisher::LogPublisher> lp = boost::make_shared<publisher::LogPublisher>( "/rosout" );
  lc->registerCallback( message_actions::PUBLISH, boost::bind(&publisher::LogPublisher::publish, lp, _1) );
  registerPublisher( lc, lp );

  /** DIAGNOSTICS */
  boost::shared_ptr<converter::DiagnosticsConverter> dc = boost::make_shared<converter::DiagnosticsConverter>( "diag", 1, sessionPtr_);
  boost::shared_ptr<publisher::BasicPublisher<diagnostic_msgs::DiagnosticArray> > dp = boost::make_shared<publisher::BasicPublisher<diagnostic_msgs::DiagnosticArray> >( "/diagnostics_agg" );
  boost::shared_ptr<recorder::DiagnosticsRecorder>   dr = boost::make_shared<recorder::DiagnosticsRecorder>( "/diagnostics_agg" );
  dc->registerCallback( message_actions::PUBLISH, boost::bind(&publisher::BasicPublisher<diagnostic_msgs::DiagnosticArray>::publish, dp, _1) );
  dc->registerCallback( message_actions::RECORD, boost::bind(&recorder::DiagnosticsRecorder::write, dr, _1) );
  dc->registerCallback( message_actions::LOG, boost::bind(&recorder::DiagnosticsRecorder::bufferize, dr, _1) );
  registerConverter( dc, dp, dr );

  /** IMU TORSO **/
  boost::shared_ptr<publisher::BasicPublisher<sensor_msgs::Imu> > imutp = boost::make_shared<publisher::BasicPublisher<sensor_msgs::Imu> >( "imu_torso" );
  boost::shared_ptr<recorder::BasicRecorder<sensor_msgs::Imu> > imutr = boost::make_shared<recorder::BasicRecorder<sensor_msgs::Imu> >( "imu_torso" );

  boost::shared_ptr<converter::ImuConverter> imutc = boost::make_shared<converter::ImuConverter>( "imu_torso", converter::IMU::TORSO, 15, sessionPtr_);
  imutc->registerCallback( message_actions::PUBLISH, boost::bind(&publisher::BasicPublisher<sensor_msgs::Imu>::publish, imutp, _1) );
  imutc->registerCallback( message_actions::RECORD, boost::bind(&recorder::BasicRecorder<sensor_msgs::Imu>::write, imutr, _1) );
  imutc->registerCallback( message_actions::LOG, boost::bind(&recorder::BasicRecorder<sensor_msgs::Imu>::bufferize, imutr, _1) );
  registerConverter( imutc, imutp, imutr );

  if(robot_type == alros::PEPPER){
    /** IMU BASE **/
    boost::shared_ptr<publisher::BasicPublisher<sensor_msgs::Imu> > imubp = boost::make_shared<publisher::BasicPublisher<sensor_msgs::Imu> >( "imu_base" );
    boost::shared_ptr<recorder::BasicRecorder<sensor_msgs::Imu> > imubr = boost::make_shared<recorder::BasicRecorder<sensor_msgs::Imu> >( "imu_base" );

    boost::shared_ptr<converter::ImuConverter> imubc = boost::make_shared<converter::ImuConverter>( "imu_base", converter::IMU::BASE, 15, sessionPtr_);
    imubc->registerCallback( message_actions::PUBLISH, boost::bind(&publisher::BasicPublisher<sensor_msgs::Imu>::publish, imubp, _1) );
    imubc->registerCallback( message_actions::RECORD, boost::bind(&recorder::BasicRecorder<sensor_msgs::Imu>::write, imubr, _1) );
    imubc->registerCallback( message_actions::LOG, boost::bind(&recorder::BasicRecorder<sensor_msgs::Imu>::bufferize, imubr, _1) );
    registerConverter( imubc, imubp, imubr );
  }

  /** Front Camera */
  boost::shared_ptr<publisher::CameraPublisher> fcp = boost::make_shared<publisher::CameraPublisher>( "camera/front/image_raw", AL::kTopCamera );
  boost::shared_ptr<recorder::CameraRecorder> fcr = boost::make_shared<recorder::CameraRecorder>( "camera/front", 2 );
  boost::shared_ptr<converter::CameraConverter> fcc = boost::make_shared<converter::CameraConverter>( "front_camera", 15, sessionPtr_, AL::kTopCamera, AL::kQVGA );
  fcc->registerCallback( message_actions::PUBLISH, boost::bind(&publisher::CameraPublisher::publish, fcp, _1, _2) );
  fcc->registerCallback( message_actions::RECORD, boost::bind(&recorder::CameraRecorder::write, fcr, _1, _2) );
  fcc->registerCallback( message_actions::LOG, boost::bind(&recorder::CameraRecorder::bufferize, fcr, _1, _2) );
  registerConverter( fcc, fcp, fcr );

  if(robot_type == alros::PEPPER){
    /** Depth Camera */
    boost::shared_ptr<publisher::CameraPublisher> dcp = boost::make_shared<publisher::CameraPublisher>( "camera/depth/image_raw", AL::kDepthCamera );
    boost::shared_ptr<recorder::CameraRecorder> dcr = boost::make_shared<recorder::CameraRecorder>( "camera/depth", 2 );
    boost::shared_ptr<converter::CameraConverter> dcc = boost::make_shared<converter::CameraConverter>( "depth_camera", 15, sessionPtr_, AL::kDepthCamera, AL::kQVGA );
    dcc->registerCallback( message_actions::PUBLISH, boost::bind(&publisher::CameraPublisher::publish, dcp, _1, _2) );
    dcc->registerCallback( message_actions::RECORD, boost::bind(&recorder::CameraRecorder::write, dcr, _1, _2) );
    dcc->registerCallback( message_actions::LOG, boost::bind(&recorder::CameraRecorder::bufferize, dcr, _1, _2) );
    registerConverter( dcc, dcp, dcr );
  }

  /** Joint States */
  boost::shared_ptr<publisher::JointStatePublisher> jsp = boost::make_shared<publisher::JointStatePublisher>( "/joint_states" );
  boost::shared_ptr<recorder::JointStateRecorder> jsr = boost::make_shared<recorder::JointStateRecorder>( "/joint_states" );
  boost::shared_ptr<converter::JointStateConverter> jsc = boost::make_shared<converter::JointStateConverter>( "joint_states", 15, tf2_buffer_, sessionPtr_ );
  jsc->registerCallback( message_actions::PUBLISH, boost::bind(&publisher::JointStatePublisher::publish, jsp, _1, _2) );
  jsc->registerCallback( message_actions::RECORD, boost::bind(&recorder::JointStateRecorder::write, jsr, _1, _2) );
  jsc->registerCallback( message_actions::LOG, boost::bind(&recorder::JointStateRecorder::bufferize, jsr, _1, _2) );
  registerConverter( jsc, jsp, jsr );
//  registerRecorder(jsc, jsr);

  if(robot_type == alros::PEPPER){
    /** Laser */
    boost::shared_ptr<publisher::BasicPublisher<sensor_msgs::LaserScan> > lp = boost::make_shared<publisher::BasicPublisher<sensor_msgs::LaserScan> >( "laser" );
    boost::shared_ptr<recorder::BasicRecorder<sensor_msgs::LaserScan> > lr = boost::make_shared<recorder::BasicRecorder<sensor_msgs::LaserScan> >( "laser" );
    boost::shared_ptr<converter::LaserConverter> lc = boost::make_shared<converter::LaserConverter>( "laser", 10, sessionPtr_ );
    lc->registerCallback( message_actions::PUBLISH, boost::bind(&publisher::BasicPublisher<sensor_msgs::LaserScan>::publish, lp, _1) );
    lc->registerCallback( message_actions::RECORD, boost::bind(&recorder::BasicRecorder<sensor_msgs::LaserScan>::write, lr, _1) );
    lc->registerCallback( message_actions::LOG, boost::bind(&recorder::BasicRecorder<sensor_msgs::LaserScan>::bufferize, lr, _1) );
    registerConverter( lc, lp, lr );
  }

  /** Sonar */
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
  boost::shared_ptr<publisher::SonarPublisher> usp = boost::make_shared<publisher::SonarPublisher>( sonar_topics );
  boost::shared_ptr<recorder::SonarRecorder> usr = boost::make_shared<recorder::SonarRecorder>( sonar_topics );
  boost::shared_ptr<converter::SonarConverter> usc = boost::make_shared<converter::SonarConverter>( "sonar", 10, sessionPtr_ );
  usc->registerCallback( message_actions::PUBLISH, boost::bind(&publisher::SonarPublisher::publish, usp, _1) );
  usc->registerCallback( message_actions::RECORD, boost::bind(&recorder::SonarRecorder::write, usr, _1) );
  usc->registerCallback( message_actions::LOG, boost::bind(&recorder::SonarRecorder::bufferize, usr, _1) );
  registerConverter( usc, usp, usr );
}

// public interface here
void Bridge::registerSubscriber( subscriber::Subscriber sub )
{
  std::vector<subscriber::Subscriber>::iterator it;
  it = std::find( subscribers_.begin(), subscribers_.end(), sub );
  size_t sub_index = 0;

  // if subscriber is not found, register it!
  if (it == subscribers_.end() )
  {
    sub_index = subscribers_.size();
    sub.reset( *nhPtr_ );
    subscribers_.push_back( sub );
    std::cout << "registered subscriber:\t" << sub.name() << std::endl;
  }
  // if found, re-init them
  else
  {
    std::cout << "re-initialized existing subscriber:\t" << it->name() << std::endl;
  }
}

void Bridge::registerDefaultSubscriber()
{
  if (!subscribers_.empty())
    return;
  registerSubscriber( boost::make_shared<alros::subscriber::TeleopSubscriber>("teleop", "/cmd_vel", sessionPtr_) );
  registerSubscriber( boost::make_shared<alros::subscriber::MovetoSubscriber>("moveto", "/move_base_simple/goal", sessionPtr_, tf2_buffer_) );
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

std::string Bridge::getMasterURI() const
{
  return ros_env::getMasterURI();
}

void Bridge::setMasterURI( const std::string& uri)
{
  setMasterURINet(uri, "eth0");
}
void Bridge::setMasterURINet( const std::string& uri, const std::string& network_interface)
{
  // To avoid two calls to this function happening at the same time
  boost::mutex::scoped_lock lock( mutex_reinit_ );

  // Stopping the loop if there is any
  stopRosLoop();

  // Reinitializing ROS Node
  {
    nhPtr_.reset();
    std::cout << "nodehandle reset " << std::endl;
    ros_env::setMasterURI( uri, network_interface );
    nhPtr_.reset( new ros::NodeHandle("~") );
  }

  if(converters_.empty())
  {
    // If there is no converters, create them
    // (converters only depends on Naoqi, resetting the
    // Ros node has no impact on them)
    registerDefaultConverter();
    registerDefaultSubscriber();
//    startRosLoop();
  }
  else
  {
    // If some converters are already there, then
    // we just need to reset the registered publisher
    // using the new ROS node handler.
    typedef std::map< std::string, publisher::Publisher > publisher_map;
    for_each( publisher_map::value_type &pub, pub_map_ )
    {
      pub.second.reset(*nhPtr_);
    }

    for_each( subscriber::Subscriber& sub, subscribers_ )
    {
      std::cout << "resetting subscriber " << sub.name() << std::endl;
      sub.reset( *nhPtr_ );
    }
  }
  if (!event_map_.empty()) {
    typedef std::map< std::string, event::Event > event_map;
    for_each( event_map::value_type &event, event_map_ )
    {
      event.second.reset(*nhPtr_, recorder_);
    }
  }
  // Start publishing again
  startRosLoop();
}

void Bridge::startPublishing()
{
  publish_enabled_ = true;
  for(EventIter iterator = event_map_.begin(); iterator != event_map_.end(); iterator++)
  {
    iterator->second.isPublishing(true);
  }
}

void Bridge::stopPublishing()
{
  publish_enabled_ = false;
  for(EventIter iterator = event_map_.begin(); iterator != event_map_.end(); iterator++)
  {
    iterator->second.isPublishing(false);
  }
}

std::vector<std::string> Bridge::getSubscribedPublishers() const
{
  std::vector<std::string> publisher;
  for(PubConstIter iterator = pub_map_.begin(); iterator != pub_map_.end(); iterator++)
  {
    // iterator->first = key
    // iterator->second = value
    // Repeat if you also want to iterate through the second map.
    if ( iterator->second.isSubscribed() )
    {
      publisher.push_back( iterator->second.topic() );
    }
  }
  return publisher;
}

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
  bool success = false;
  int index = 0;
  while (!success && (index<names.size()))
  {
    if (rec_map_.find(names[index]) != rec_map_.end())
    {
      success = true;
    }
    ++index;
  }

  if (success)
  {
    boost::mutex::scoped_lock lock_record( mutex_record_ );
    recorder_->startRecord();

    for_each( const std::string& name, names)
    {
      RecIter it = rec_map_.find(name);
      if ( it != rec_map_.end() )
      {
        it->second.subscribe(true);
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
  // Create the publishing thread if needed
  keep_looping = true;
  if (publisherThread_.get_id() ==  boost::thread::id())
    publisherThread_ = boost::thread( &Bridge::rosLoop, this );
  for(EventIter iterator = event_map_.begin(); iterator != event_map_.end(); iterator++)
  {
    iterator->second.startProcess();
  }
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
  // Check if the nodeHandle pointer is already initialized
  if(!nhPtr_){
    std::cout << BOLDRED << "The connection with the ROS master does not seem to be initialized." << std::endl
              << BOLDYELLOW << "Please run:" << RESETCOLOR << std::endl
              << GREEN << "\t$ qicli call BridgeService.setMasterURI <YourROSCoreIP>" << RESETCOLOR << std::endl
              << BOLDYELLOW << "before trying to add converters" << RESETCOLOR << std::endl;
    return;
  }

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
  boost::shared_ptr<publisher::BasicPublisher<naoqi_bridge_msgs::MemoryList> > mlp = boost::make_shared<publisher::BasicPublisher<naoqi_bridge_msgs::MemoryList> >( topic );
  boost::shared_ptr<recorder::BasicRecorder<naoqi_bridge_msgs::MemoryList> > mlr = boost::make_shared<recorder::BasicRecorder<naoqi_bridge_msgs::MemoryList> >( topic );
  boost::shared_ptr<converter::MemoryListConverter> mlc = boost::make_shared<converter::MemoryListConverter>(list, topic, frequency, sessionPtr_ );
  mlc->registerCallback( message_actions::PUBLISH, boost::bind(&publisher::BasicPublisher<naoqi_bridge_msgs::MemoryList>::publish, mlp, _1) );
  mlc->registerCallback( message_actions::RECORD, boost::bind(&recorder::BasicRecorder<naoqi_bridge_msgs::MemoryList>::write, mlr, _1) );
  registerConverter( mlc, mlp, mlr );
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
      boost::shared_ptr<EventRegister<converter::MemoryFloatConverter,publisher::BasicPublisher<naoqi_bridge_msgs::FloatStamped>,recorder::BasicRecorder<naoqi_bridge_msgs::FloatStamped> > > event_register =
          boost::make_shared<EventRegister<converter::MemoryFloatConverter,publisher::BasicPublisher<naoqi_bridge_msgs::FloatStamped>,recorder::BasicRecorder<naoqi_bridge_msgs::FloatStamped> > >( key, sessionPtr_ );
      insertEventConverter(key, event_register);
      break;
    }
  case 2:
    {
      boost::shared_ptr<EventRegister<converter::MemoryIntConverter,publisher::BasicPublisher<naoqi_bridge_msgs::IntStamped>,recorder::BasicRecorder<naoqi_bridge_msgs::IntStamped> > > event_register =
          boost::make_shared<EventRegister<converter::MemoryIntConverter,publisher::BasicPublisher<naoqi_bridge_msgs::IntStamped>,recorder::BasicRecorder<naoqi_bridge_msgs::IntStamped> > >( key, sessionPtr_ );
      insertEventConverter(key, event_register);
      break;
    }
  case 3:
    {
      boost::shared_ptr<EventRegister<converter::MemoryStringConverter,publisher::BasicPublisher<naoqi_bridge_msgs::StringStamped>,recorder::BasicRecorder<naoqi_bridge_msgs::StringStamped> > > event_register =
          boost::make_shared<EventRegister<converter::MemoryStringConverter,publisher::BasicPublisher<naoqi_bridge_msgs::StringStamped>,recorder::BasicRecorder<naoqi_bridge_msgs::StringStamped> > >( key, sessionPtr_ );
      insertEventConverter(key, event_register);
      break;
    }
  case 4:
    {
      boost::shared_ptr<EventRegister<converter::MemoryBoolConverter,publisher::BasicPublisher<naoqi_bridge_msgs::BoolStamped>,recorder::BasicRecorder<naoqi_bridge_msgs::BoolStamped> > > event_register =
          boost::make_shared<EventRegister<converter::MemoryBoolConverter,publisher::BasicPublisher<naoqi_bridge_msgs::BoolStamped>,recorder::BasicRecorder<naoqi_bridge_msgs::BoolStamped> > >( key, sessionPtr_ );
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
  if (publish_enabled_) {
    event_map_.find(key)->second.isPublishing(true);
  }

  return true;
}

QI_REGISTER_OBJECT( Bridge,
                    _whoIsYourDaddy,
                    minidump,
                    setBufferDuration,
                    startPublishing,
                    stopPublishing,
                    getMasterURI,
                    setMasterURI,
                    setMasterURINet,
                    getAvailableConverters,
                    getSubscribedPublishers,
                    addMemoryConverters,
                    registerMemoryConverter,
                    registerEventConverter,
                    startRecording,
                    startRecordingConverters,
                    stopRecording );
} //alros
