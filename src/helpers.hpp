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

#include <boost/filesystem.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/algorithm/string/replace.hpp>

#include <alrosbridge/tools.hpp>
#include <qi/session.hpp>

#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace alros
{
namespace helpers
{

static const long folderMaximumSize = 2000000000;

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

inline void getFoldersize(std::string rootFolder, long& file_size){
  boost::algorithm::replace_all(rootFolder, "\\\\", "\\");
  boost::filesystem::path folderPath(rootFolder);
  if (boost::filesystem::exists(folderPath)){
    boost::filesystem::directory_iterator end_itr;

    for (boost::filesystem::directory_iterator dirIte(rootFolder); dirIte != end_itr; ++dirIte )
    {
      boost::filesystem::path filePath(dirIte->path());
      try{
        if (!boost::filesystem::is_directory(dirIte->status()) )
        {
          file_size = file_size + boost::filesystem::file_size(filePath);
        }else{
          getFoldersize(filePath.string(),file_size);
        }
      }catch(std::exception& e){
        std::cout << e.what() << std::endl;
      }
    }
  }
}

static const float bufferDefaultDuration = 10.f;
inline void getFiles(const boost::filesystem::path& root, const std::string& ext, std::vector<boost::filesystem::path>& ret)
{
  if(!boost::filesystem::exists(root) || !boost::filesystem::is_directory(root)) return;

  boost::filesystem::recursive_directory_iterator it(root);
  boost::filesystem::recursive_directory_iterator endit;

  while(it != endit)
  {
    if(boost::filesystem::is_regular_file(*it) && it->path().extension() == ext)
    {
      ret.push_back(it->path().filename());
    }
    ++it;
  }
}

static const std::string boot_config_file_name = "boot_config.json";

inline std::string& getBootConfigFile()
{
  static std::string path = qi::path::findData("/", boot_config_file_name );
  return path;
}

inline void getFilesSize(const boost::filesystem::path& root, long& file_size)
{
  std::vector<boost::filesystem::path> files_path;
  getFiles(root, ".bag", files_path);
  for (std::vector<boost::filesystem::path>::const_iterator it=files_path.begin();
       it!=files_path.end(); it++)
  {
    try{
      file_size = file_size + boost::filesystem::file_size(*it);
    }catch(std::exception& e){
      std::cout << e.what() << std::endl;
    }
  }
}

} //helpers
} // alros

#endif
