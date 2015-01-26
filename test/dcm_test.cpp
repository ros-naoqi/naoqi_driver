#include <ros/ros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>

#include <iostream>

#include <stdlib.h>
#include <vector>
#include <string>

int main( int argc, char** argv )
{

  setenv( "ROS_MASTER_URI", "http://10.0.132.69:11311", 1 );
  ros::init( argc, argv, "pepper_dcm_test" );
  ros::NodeHandle n;


  if(!ros::master::check())
  {
      std::cerr<<"Could not contact master!\nQuitting... "<< std::endl;
      return -1;
  }

  // load urdf model from param server
  std::vector< std::string > joint_names;
  try{
    std::string robot_description;
    n.getParam("robot_description", robot_description);
    ROS_INFO_STREAM("robot description loaded !!");

    KDL::Tree tree;
    kdl_parser::treeFromString(robot_description, tree);
    KDL::SegmentMap segment_map = tree.getSegments();

    typedef KDL::SegmentMap::const_iterator kdl_iter;
    for (kdl_iter it = segment_map.begin(); it != segment_map.end(); ++it)
    {
      // filter out non-actuated joints
      if (it->second.segment.getJoint().getType() != KDL::Joint::None)
      {
        ROS_INFO_STREAM("joint found: " << it->second.segment.getJoint().getName()
                        << it->second.segment.getJoint().getType()) ;
        joint_names.push_back(it->second.segment.getJoint().getName());
        }
    }
  }
  catch (std::exception e)
  {
    ROS_ERROR_STREAM("no robot descption found ... exiting! ");
    ROS_ERROR_STREAM(e.what());
    return -1;
  }

  std::cout << "*****list of joints*********" << std::endl;
  for (size_t i=0; i<joint_names.size(); ++i)
  {
    std::cout << "found joint\t" << joint_names[i] << std::endl;
  }

}
