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


#ifndef ALROS_TOOLS_HPP
#define ALROS_TOOLS_HPP

#define RESETCOLOR "\033[0m"
#define GREEN "\033[32m"
#define HIGHGREEN "\033[92m"
#define BOLDRED "\033[1m\033[31m"
#define YELLOW "\033[33m"
#define BOLDYELLOW "\033[1m\033[33m"
#define BOLDCYAN "\033[1m\033[36m"

# include <qi/anyobject.hpp>

namespace naoqi
{

namespace robot
{
enum Robot
{
  UNIDENTIFIED,
  NAO,
  PEPPER,
  ROMEO
};
}

enum Topics {
  Laser = 0,
  Camera,
  Sonar
};

namespace dataType {
enum DataType
{
  None = 0,
  Float,
  Int,
  String,
  Bool
};
}

} // naoqi

QI_TYPE_ENUM_REGISTER(naoqi::Topics);
QI_TYPE_ENUM_REGISTER(naoqi::dataType::DataType);

#endif
