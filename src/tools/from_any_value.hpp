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

#ifndef FROM_ANY_VALUE_HPP
#define FROM_ANY_VALUE_HPP

/*
* LOCAL includes
*/
#include "naoqi_image.hpp"

/*
* ALDEBARAN includes
*/
#include <qi/anyvalue.hpp>

namespace naoqi {

namespace tools {

NaoqiImage fromAnyValueToNaoqiImage(qi::AnyValue& value);

std::vector<std::string> fromAnyValueToStringVector(qi::AnyValue& value, std::vector<std::string>& result);

std::vector<float> fromAnyValueToFloatVector(qi::AnyValue& value, std::vector<float>& result);

std::vector<int> fromAnyValueToIntegerVector(qi::AnyValue& value, std::vector<int>& result);

std::vector<std::pair<std::string, bool> > fromAnyValueToStringBoolPairVector(qi::AnyValue& value, std::vector<std::pair<std::string, bool> >& result);

}

}

#endif // FROM_ANY_VALUE_HPP
