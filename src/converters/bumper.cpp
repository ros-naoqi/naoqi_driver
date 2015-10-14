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
* LOCAL includes
*/
#include "bumper.hpp"
#include "../tools/from_any_value.hpp"

/*
* ROS includes
*/
//#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

/*
* BOOST includes
*/
#include <boost/foreach.hpp>
#define for_each BOOST_FOREACH

namespace naoqi
{
namespace converter {


  BumperConverter::BumperConverter(const std::string& name, const float& frequency, const qi::SessionPtr& session):
    BaseConverter(name, frequency, session),
    p_touch_( session->service("ALTouch") )
  {
  }

  BumperConverter::~BumperConverter()
  {
  }

  void BumperConverter::reset()
  {

  }

  void BumperConverter::registerCallback( const message_actions::MessageAction action, Callback_t cb )
  {
    callbacks_[action] = cb;
  }

  void BumperConverter::callAll(const std::vector<message_actions::MessageAction>& actions)
  {
    // Get inertial data
    std::vector<std::pair<std::string, bool> > values;
    try {
      qi::AnyValue anyvalues = p_touch_.call<qi::AnyValue>("getStatus");
      tools::fromAnyValueToStringBoolPairVector(anyvalues, values);
    } catch (const std::exception& e) {
      std::cerr << "Exception caught in BumperConverter: " << e.what() << std::endl;
      return;
    }

    int i = 0;
    for (std::vector<std::pair<std::string, bool> >::const_iterator it=values.begin();
         it!=values.end(); it++, i++)
    {
      std::string name = it->first;
      bool state = it->second;
      if ( name == "Bumper/FrontLeft" ) {
        msg_bumper_.bumper = naoqi_bridge_msgs::Bumper::left;
        msg_bumper_.state = state?(naoqi_bridge_msgs::Bumper::statePressed):(naoqi_bridge_msgs::Bumper::stateReleased);
      } else if ( name == "Bumper/FrontLeft" ) {
        msg_bumper_.bumper = naoqi_bridge_msgs::Bumper::right;
        msg_bumper_.state = state?(naoqi_bridge_msgs::Bumper::statePressed):(naoqi_bridge_msgs::Bumper::stateReleased);
      }
    }

    for_each( message_actions::MessageAction action, actions )
    {
      callbacks_[action]( msg_bumper_ );
    }
  }

}

}
