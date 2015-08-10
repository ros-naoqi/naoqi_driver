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
#include <naoqi_driver/naoqi_driver.hpp>

/*
 * ALDEBARAN includes
 */
#include <qi/anymodule.hpp>

/**
* @brief starter code for registrating the naoqi_driver module via the autoload.ini.
*/
void registerRosDriver(qi::ModuleBuilder* mb) {
  mb->advertiseFactory<naoqi::Driver, qi::SessionPtr, std::string>("ROS-Driver");
}
QI_REGISTER_MODULE("naoqi_driver_module", &registerRosDriver);
