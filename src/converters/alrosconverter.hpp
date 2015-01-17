#include <string>

// QI concurrency
//#include <qi/actor.hpp>

/*
* ROS dependencies
* comes with boost
*/
#include <ros/ros.h>
#include <boost/scoped_ptr.hpp>
#include <boost/thread/mutex.hpp>


namespace alros
{

class ALRosConverter //: public qi::Actor
{
  public:
    ALRosConverter( int argc, char** argv, const std::string& ip );

    /**
    * callable with qicli call
    */
    std::string getMasterURI() const;

    /**
    * callable with qicli call
    */
    void setMasterURI( const std::string& uri );

    /**
    * @brief: isAlive function for polling state
    * @return: bool for alive state
    */
    bool isAlive() const;

    /**
    * @brief: update function to do logic
    */
    void update();

  private:
    /*
    * @brief: init all ros components needed such as nodehandle and publisher
    */
    void initPublisher();

    boost::scoped_ptr<ros::NodeHandle> nhPtr_;
    ros::Publisher pub_;

    boost::mutex mutex_reinit_;

};

} //alros
