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

#include <alrosbridge/recorder/recorder.hpp>
#include <std_msgs/Int32.h>

#include <qi/log.hpp>

qiLogCategory("ros.Recorder");

namespace alros
{

  Recorder::Recorder():
    _bag()
  , _processMutex()
  , _nameBag("out.bag")
  , _isRecording(false)
  {

  }

  void Recorder::startRecord() {
    boost::mutex::scoped_lock startLock( _processMutex );
    if (!_isRecording) {
      try {
        _bag.open(_nameBag, rosbag::bagmode::Write);
        _isRecording = true;
        std::cout << "The bag " << _nameBag << " is opened !" << std::endl;
      } catch (std::exception e){
        throw std::runtime_error(e.what());
      }
    }
    else {
      qiLogError() << "Cannot start a record. The module is already recording.";
    }
  }

  void Recorder::startRecordTopics(const std::vector<Topics>& topics) {
    boost::mutex::scoped_lock startLock( _processMutex );
    if (!_isRecording) {
      try {
        _bag.open(_nameBag, rosbag::bagmode::Write);
        _isRecording = true;
        std::cout << "The bag " << _nameBag << " is opened !" << std::endl;
      } catch (std::exception e){
        throw std::runtime_error(e.what());
      }
    }
    else {
      qiLogError() << "Cannot start a record. The module is already recording.";
    }

  }

  void Recorder::stopRecord() {
    boost::mutex::scoped_lock stopLock( _processMutex );
    if (_isRecording) {
      _bag.close();
      _isRecording = false;
      std::cout << "The bag " << _nameBag << " is closed !" << std::endl;
    }
    else {
      qiLogError() << "Cannot stop recording while it has not been started.";
    }
  }

  bool Recorder::isRecording() {
    return _isRecording;
  }

}
