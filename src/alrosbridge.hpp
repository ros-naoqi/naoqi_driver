#ifndef ALROS_BRIDGE_HPP
#define ALROS_BRIDGE_HPP

#include <vector>

/*
* BOOST
*/
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/scoped_ptr.hpp>

/*
* ALDEB
*/
#include <qi/session.hpp>


#include "publishers/publisher.hpp"
namespace alros
{

class Bridge
{
public:
  Bridge( qi::SessionPtr& session );
  ~Bridge();

  // make a copy here since this should actually be replaced by move semantics
  void registerPublisher( publisher::Publisher pub );

  void publish( );

  bool isAlive() const;

  /**
  * callable with qicli call
  */
  std::string getMasterURI() const;
  void setMasterURI( const std::string& uri );
  void start();
  void stop();


private:
  qi::SessionPtr sessionPtr_;
  bool publish_enabled_;
  const size_t freq_;
  boost::thread publisherThread_;
  //ros::Rate r_;

  void registerDefaultPublisher();
  void initPublisher();

  void rosLoop();

  boost::scoped_ptr<ros::NodeHandle> nhPtr_;
  boost::mutex mutex_reinit_;

  std::vector< publisher::Publisher > all_publisher_;
};

} // alros

#endif
