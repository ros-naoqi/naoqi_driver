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


#ifndef HELPERS_HPP
#define HELPERS_HPP

#include <alrosbridge/publisher/publisher.hpp>
#include <alrosbridge/tools.hpp>
#include <qi/session.hpp>

namespace alros
{
namespace helpers
{

inline bool hasSameTopic( const publisher::Publisher& first, const publisher::Publisher& second )
{
  if ( first.topic() == second.topic() )
    return true;
  else
    return false;
}

inline dataType::DataType getDataType(qi::AnyValue value)
{
  dataType::DataType type;
  if (value.kind() == qi::TypeKind_Int) {
    type = dataType::Int;
  }
  else if (value.kind() == qi::TypeKind_Float) {
    type = dataType::Float;
  }
  else if (value.kind() == qi::TypeKind_String) {
    type = dataType::String;
  }
  else {
    throw std::runtime_error("Cannot get a valid type.");
  }
  return type;
}

static const float bufferDefaultDuration = 10.f;

static const std::string boot_config_file_name = "boot_config.json";

inline std::string& getBootConfigFile()
{
  static std::string path = qi::path::findData("/", boot_config_file_name );
  return path;
}

} //helpers
} // alros

#endif
