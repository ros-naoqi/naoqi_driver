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


#ifndef FILESYSTEM_HELPERS_HPP
#define FILESYSTEM_HELPERS_HPP

#include <qi/session.hpp>

#include <boost/filesystem.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/algorithm/string/replace.hpp>

#ifdef CATKIN_BUILD
#include <ros/package.h>
#endif

namespace naoqi
{
namespace helpers
{
namespace filesystem
{

static const long folderMaximumSize = 2000000000;

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

/** Boot config loader */
static const std::string boot_config_file_name = "boot_config.json";
inline std::string& getBootConfigFile()
{
#ifdef CATKIN_BUILD
  static std::string path = ros::package::getPath("naoqi_driver")+"/share/"+boot_config_file_name;
  std::cout << "found a catkin prefix " << path << std::endl;
  return path;
#else
  static std::string path = qi::path::findData( "/", boot_config_file_name );
  std::cout << "found a qibuild path " << path << std::endl;
  return path;
#endif
}

/* URDF loader */
inline std::string& getURDF( std::string filename )
{
#ifdef CATKIN_BUILD
  static std::string path = ros::package::getPath("naoqi_driver")+"/share/urdf/"+filename;
  std::cout << "found a catkin URDF " << path << std::endl;
  return path;
#else
  static std::string path = qi::path::findData( "/urdf/", filename );
  std::cout << "found a qibuild URDF " << path << std::endl;
  return path;
#endif
}

} // filesystem
} //helpers
} // naoqi

#endif
