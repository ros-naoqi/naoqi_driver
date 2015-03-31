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

#include <alrosbridge/recorder/globalrecorder.hpp>
#include <std_msgs/Int32.h>
#include <tf2_msgs/TFMessage.h>
#include <qi/log.hpp>
#include <ctime>

#define RESET "\033[0m"
#define GREEN "\033[32m"             /* Green */
#define BOLDRED "\033[1m\033[31m"    /* Bold Red */

qiLogCategory("ros.Recorder");

namespace alros
{
namespace recorder
{

  GlobalRecorder::GlobalRecorder():
    _bag()
  , _processMutex()
  , _nameBag("")
  , _isStarted(false)
  {
  }

  void GlobalRecorder::startRecord() {
    boost::mutex::scoped_lock startLock( _processMutex );
    if (!_isStarted) {
      try {
        time_t rawtime;
        struct tm * timeinfo;
        char buffer[80];
        std::time(&rawtime);
        timeinfo = std::localtime(&rawtime);
        std::strftime(buffer,80,"%d-%m-%Y_%I:%M:%S",timeinfo);
        _nameBag = buffer;
        _nameBag.append(".bag");

        _bag.open(_nameBag, rosbag::bagmode::Write);
        _isStarted = true;
        std::cout << "The bag " << _nameBag << " is opened" << std::endl;
      } catch (std::exception e){
        throw std::runtime_error(e.what());
      }
    }
    else {
      qiLogError() << "Cannot start a record. The module is already recording.";
    }
  }

  void GlobalRecorder::stopRecord(const std::string& robot_ip) {
    boost::mutex::scoped_lock stopLock( _processMutex );
    if (_isStarted) {
      _bag.close();
      _isStarted = false;
      std::cout << "The bag " << _nameBag << " is closed" << std::endl;
      std::cout << BOLDRED << "To download this bag on your computer:" << RESET << std::endl
                   << GREEN << "\t$ scp nao@" << robot_ip << ":/home/nao/" << _nameBag << " <LOCAL_PATH>" << RESET
                      << std::endl;
      _nameBag.clear();
    }
    else {
      qiLogError() << "Cannot stop recording while it has not been started.";
    }
  }

  bool GlobalRecorder::isStarted() {
    return _isStarted;
  }

  void GlobalRecorder::write(const std::string& topic, const std::vector<geometry_msgs::TransformStamped>& msgtf) {
    tf2_msgs::TFMessage message;
    ros::Time now = ros::Time::now();
    if (!msgtf[0].header.stamp.isZero()) {
      now = msgtf[0].header.stamp;
    }
    for (std::vector<geometry_msgs::TransformStamped>::const_iterator it = msgtf.begin(); it != msgtf.end(); ++it)
    {
      message.transforms.push_back(*it);
    }
    boost::mutex::scoped_lock writeLock( _processMutex );
    if (_isStarted) {
      _bag.write(topic, now, message);
    }
  }

} // recorder
} // alros
