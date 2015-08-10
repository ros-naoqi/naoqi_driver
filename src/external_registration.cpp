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


#include <qi/applicationsession.hpp>
#include <qi/anymodule.hpp>
#include <naoqi_driver/naoqi_driver.hpp>

#include <boost/program_options.hpp>

#include "naoqi_env.hpp"
#include "helpers/driver_helpers.hpp"

int main(int argc, char** argv)
{
  /* adjust the SDK prefix in case you compiled via catkin*/
  naoqi::naoqi_env::adjustSDKPrefix();

  /* launch naoqi service */
  qi::ApplicationSession app(argc, argv);
  /* In case you launch via roslaunch/rosrun we remove the ros args */
  std::vector<std::string> args_out;
  ros::removeROSArgs( argc, argv, args_out );

  namespace po = boost::program_options;
  po::options_description desc("Options");
  desc.add_options()
    ("help,h", "print help message")
    ("roscore_ip,r", po::value<std::string>(), "set the ip of the roscore to use")
    ("network_interface,i", po::value<std::string>()->default_value("eth0"),  "set the network interface over which to connect")
    ("namespace,n", po::value<std::string>()->default_value(""), "set an explicit namespace in case ROS namespace variables cannot be used");

  po::variables_map vm;
  try
  {
    po::store( po::parse_command_line(argc, argv, desc), vm );
  }
  catch (boost::program_options::invalid_command_line_syntax& e)
  {
    std::cout << "error " << e.what() << std::endl;
    throw ros::Exception(e.what());
  }
  catch (boost::program_options::unknown_option& e)
  {
    std::cout << "error 2 " << e.what() << std::endl;
    throw ros::Exception(e.what());
  }

  if( vm.count("help") )
  {
    std::cout << "This is the help message for the ROS-Driver C++ bridge" << std::endl << desc << std::endl;
    exit(0);
  }


  // everything's correctly parsed - let's start the app!
  app.start();
  boost::shared_ptr<naoqi::Driver> bs = qi::import("naoqi_driver_module").call<qi::Object<naoqi::Driver> >("ROS-Driver", app.session(), vm["namespace"].as<std::string>()).asSharedPtr();

  app.session()->registerService("ROS-Driver", bs);


  // set ros paramters directly upfront if available
  if ( vm.count("roscore_ip") )
  {
    std::string roscore_ip = vm["roscore_ip"].as<std::string>();
    std::string network_interface = vm["network_interface"].as<std::string>();

    std::cout << BOLDYELLOW << "using ip address: "
              << BOLDCYAN << roscore_ip << " @ " << network_interface << RESETCOLOR << std::endl;
    bs->init();
    bs->setMasterURINet( "http://"+roscore_ip+":11311", network_interface);
  }
  else
  {
    std::cout << BOLDRED << "No ip address given. Run qicli call to set the master uri" << RESETCOLOR << std::endl;
    bs->init();
  }

  //! @note Must call ow._stopService when the application stops to do the clean-up
  app.atStop(boost::function<void()>(
                 boost::bind(&naoqi::Driver::stopService, boost::ref(bs))));

  app.run();
  app.session()->close();
}
