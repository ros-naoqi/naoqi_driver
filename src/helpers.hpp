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

namespace alros
{
namespace helpers
{

inline bool hasSameName( const publisher::Publisher& first, const publisher::Publisher& second )
{
  if ( first.name() == second.name() )
    return true;
  else
    return false;
}



} //helpers
} // alros

#endif
