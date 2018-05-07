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

#ifndef TOUCH_EVENT_CONVERTER_HPP
#define TOUCH_EVENT_CONVERTER_HPP

/*
* LOCAL includes
*/
#include "converter_base.hpp"
#include <naoqi_driver/message_actions.h>

/*
* ROS includes
*/
#include <naoqi_bridge_msgs/Bumper.h>
#include <naoqi_bridge_msgs/HandTouch.h>
#include <naoqi_bridge_msgs/HeadTouch.h>
#include <naoqi_bridge_msgs/ChestButtonPressed.h>

/*
* ALDEBARAN includes
*/
#include <qi/anymodule.hpp>

namespace naoqi{

namespace converter{

template <class T>
class TouchEventConverter : public BaseConverter<TouchEventConverter<T> >
{

  typedef boost::function<void(T&) > Callback_t;

public:
  TouchEventConverter(const std::string& name, const float& frequency, const qi::SessionPtr& session);

  ~TouchEventConverter();

  virtual void reset();

  void registerCallback( const message_actions::MessageAction action, Callback_t cb );

  void callAll(const std::vector<message_actions::MessageAction>& actions, T& msg);

private:
  /** Registered Callbacks **/
  std::map<message_actions::MessageAction, Callback_t> callbacks_;
  T msg_;
};

}

}

#endif // TOUCH_CONVERTER_HPP
