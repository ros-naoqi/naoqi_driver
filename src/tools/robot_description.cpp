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
#include "robot_description.hpp"
#include "../helpers/filesystem_helpers.hpp"

namespace naoqi{

namespace tools{

std::string getRobotDescription( const robot::Robot& robot){
    std::string urdf_path;
    static std::string robot_desc;
    if(!robot_desc.empty())
      return robot_desc;

    if ( robot == robot::PEPPER)
    {
      urdf_path = helpers::filesystem::getURDF("pepper.urdf");
    }
    else if ( robot == robot::NAO )
    {
      urdf_path = helpers::filesystem::getURDF("nao.urdf");
    }
    else
    {
      std::cerr << " could not load urdf file from disk " << std::endl;
      return std::string();
    }

    std::ifstream stream( (urdf_path).c_str() );
    if (!stream)
    {
      std::cerr << "failed to load robot description in joint_state_publisher: " << urdf_path << std::endl;
      return std::string();
    }
    robot_desc = std::string( (std::istreambuf_iterator<char>(stream)), std::istreambuf_iterator<char>());
    return robot_desc;
}

} // tools

} // naoqi
