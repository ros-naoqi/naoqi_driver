#ifndef ALROS_BRIDGE_HPP
#define ALROS_BRIDGE_HPP

#include <vector>

/*
* BOOST
*/
#include <boost/thread/mutex.hpp>
#include <boost/scoped_ptr.hpp>


#include "publishers/publisher.hpp"
namespace alros
{

class Bridge
{
public:
  Bridge( const std::string& ip );
  ~Bridge();

  /**
  * callable with qicli call
  */
  std::string getMasterURI() const;

  /**
  * callable with qicli call
  */
  void setMasterURI( const std::string& uri );

  // make a copy here since this should actually be replaced by move semantics
  void registerPublisher( publisher::Publisher pub );

  void publish( );

  bool isAlive() const;

private:

  void initPublisher();

  boost::scoped_ptr<ros::NodeHandle> nhPtr_;
  boost::mutex mutex_reinit_;

  std::vector< publisher::Publisher > all_publisher_;
};

} // alros

#endif
