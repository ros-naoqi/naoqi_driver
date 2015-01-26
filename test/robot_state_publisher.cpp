#include <ros/ros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <robot_state_publisher/robot_state_publisher.h>

#include <iostream>
#include <stdlib.h>
#include <vector>
#include <string>



int main( int argc, char** argv )
{

  setenv( "ROS_MASTER_URI", "http://10.0.132.69:11311", 1 );
  ros::init( argc, argv, "robot_state_publisher_test" );
  ros::NodeHandle n;


  if(!ros::master::check())
  {
      std::cerr<<"Could not contact master!\nQuitting... "<< std::endl;
      return -1;
  }

  // load urdf model from param server
  std::vector< std::string > joint_names;
  KDL::Tree tree;
  try{
    std::string robot_description;
    n.getParam("robot_description", robot_description);
    ROS_INFO_STREAM("robot description loaded !!");

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

  std::map< std::string, double> joint_states;
  for ( size_t i=0; i<joint_names.size(); ++i)
  {
    joint_states[joint_names[i]] = 0.0;
  }
  std::cout << "*****list of joints*********" << std::endl;
  for (size_t i=0; i<joint_names.size(); ++i)
  {
    std::cout << "found joint\t" << joint_names[i] << std::endl;
  }

  // instantiate the robot state publisher
  robot_state_publisher::RobotStatePublisher rsp( tree );
  while( ros::ok() )
  {
    rsp.publishTransforms( joint_states, ros::Time::now(), "" );
    rsp.publishFixedTransforms( "" );
  }

}
