/*
 * ROS
 */
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/image_encodings.h>
#include <naoqi_bridge_msgs/BoolStamped.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <stdlib.h>
#include <cstdio>
#include <vector>
#include <string>
#include <boost/assign/list_of.hpp>
#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/algorithm/string/replace.hpp>

#include <qi/os.hpp>
#include <qi/applicationsession.hpp>

#include <alrosbridge/tools.hpp>
#include <alrosbridge/recorder/recorder.hpp>
#include <alrosbridge/recorder/globalrecorder.hpp>

#include "../src/recorder/basic.hpp"
#include "../src/converters/memory/float.hpp"
#include "../src/converters/memory/bool.hpp"

#include "../src/event/basic.hpp"

void getFoldersize(std::string rootFolder, long& file_size){
  boost::algorithm::replace_all(rootFolder, "\\\\", "\\");
  boost::filesystem::path folderPath(rootFolder);
  //boost::filesystem::path folderPath( boost::filesystem::current_path() );
  if (boost::filesystem::exists(folderPath)){
    boost::filesystem::directory_iterator end_itr;

    for (boost::filesystem::directory_iterator dirIte(rootFolder); dirIte != end_itr; ++dirIte )
    //for (boost::filesystem::directory_iterator dirIte(folderPath.string()); dirIte != end_itr; ++dirIte )
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

void get_files(const boost::filesystem::path& root, const std::string& ext, std::vector<boost::filesystem::path>& ret)
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

void getFilesSize(std::vector<boost::filesystem::path> ret, long& file_size)
{
  for (std::vector<boost::filesystem::path>::const_iterator it=ret.begin();
       it!=ret.end(); it++)
  {
    try{
      file_size = file_size + boost::filesystem::file_size(*it);
    }catch(std::exception& e){
      std::cout << e.what() << std::endl;
    }
  }
}

int main( int argc, char** argv )
{
  qi::ApplicationSession app(argc, argv);
  app.start();

  // GET CURRENT DIRECTORY PATH
  long size = 0;
  boost::filesystem::path folderPath( boost::filesystem::current_path() );
  getFoldersize(folderPath.string(), size);
  std::cout << "Size of folder " << folderPath.string()
            << ": " << size << std::endl;

  std::vector<boost::filesystem::path> ret;
  get_files(folderPath, ".bag", ret);
  for (std::vector<boost::filesystem::path>::const_iterator it=ret.begin();
       it!=ret.end(); it++)
  {
    std::cout << it->string() << std::endl;
  }

  long size_bags = 0;
  getFilesSize(ret, size_bags);
  std::cout << "Size of files " << folderPath.string()
            << ": " << size_bags << std::endl;

  //std::remove(ret.front().c_str());

  /*qi::SessionPtr sessionPtr = app.session();

  float buffer_duration = 10.f;
  size_t buffer_size = static_cast<size_t>(buffer_duration * 2.f);

  std::cout << "DATA:\t - duration = " << buffer_duration << std::endl
            << "\t - size = " << buffer_size << std::endl;

  buffer_size = ( buffer_size / buffer_duration ) * 20.f;
  buffer_duration = 20.f;

  std::cout << "DATA:\t - duration = " << buffer_duration << std::endl
            << "\t - size = " << buffer_size << std::endl;*/

  app.session()->close();
  return 0;

}
