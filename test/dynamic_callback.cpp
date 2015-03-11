/**
 * test for having a cyclic loop and connecting those items with various callbacks
 */

#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#define for_each BOOST_FOREACH
/*
 * ROS
 */
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>

/**
 * Emulate different publishers here
 */
struct StringPublisher
{
  void publish( const std_msgs::String& m )
  {
    std::cout << "String Publisher publishes " << m.data << std::endl;
  }
};
struct Float32Publisher
{
  void publish( const std_msgs::Float32& m )
  {
    std::cout << "Float32 Publisher publishes " << m.data << std::endl;
  }
};
struct Int32Publisher
{
  void publish( const std_msgs::Int32& m1, const std_msgs::Int32& m2 )
  {
    std::cout << "Int Publisher publishes two messages " << m1.data << " and " << m2.data << std::endl;
  }
};

/**
 * Emulate the recorder
 */
struct Recorder
{
  template <typename T>
  void record( const T& m )
  {
    std::cout << "Recorder records " << m.data << std::endl;
  }

  template <typename T>
  void record( const std_msgs::Int32& m1, const std_msgs::Int32& m2 )
  {
    std::cout << "Recorder writes directly two cool InterMsg " << m1.data << " and " << m2.data << std::endl;
  }
};


enum MessageAction
{
  PUBLISH,
  RECORD,
  LOG
};

/**
 * Emulate the translation unit
 */
struct StringConverter
{
  typedef boost::function<void(std_msgs::String)> Callback;

  std::map<MessageAction, Callback> callbacks;

  StringConverter()
  {}

  void registerCallback( MessageAction action, Callback cb )
  {
    callbacks[action] = cb;
  }

  void operator()( const std::vector<MessageAction>& actions )
  {
    std_msgs::String m;
    m.data = "Hello String";

    for_each( const MessageAction& action, actions )
    {
      callbacks[action](m);
    }
  }
};

struct Float32Converter
{
  typedef boost::function<void(std_msgs::Float32)> Callback;

  std::map<MessageAction, Callback> callbacks;

  Float32Converter()
  {}

  void registerCallback( MessageAction action, Callback cb )
  {
    callbacks[action] = cb;
  }

  void operator()( const std::vector<MessageAction>& actions )
  {
    std_msgs::Float32 m;
    m.data = 3.14561;

    for_each( const MessageAction& action, actions )
    {
      callbacks[action](m);
    }
  }
};

/*
 * make the map templated by the callback functions
 * typedef booost::function<void(std_msgs::Int32, std_msgs::Int32)> Callback
 * CallbackMap<Callback> callbacks
 */

struct Int32Converter
{
  typedef boost::function<void(std_msgs::Int32, std_msgs::Int32)> Callback;

  std::map<MessageAction, Callback> callbacks;

  Int32Converter()
  {}

  void registerCallback( MessageAction action, Callback cb )
  {
    callbacks[action] = cb;
  }

  void operator()( const std::vector<MessageAction>& actions )
  {
    std_msgs::Int32 m1;
    m1.data = 123;

    std_msgs::Int32 m2;
    m2.data = 456;

    for_each ( MessageAction action, actions )
    {
      callbacks[action]( m1, m2 );
    }
  }
};


int main()
{
  StringPublisher sp;
  Float32Publisher fp;
  Int32Publisher ip;

  Recorder rec;

  StringConverter sc;
  sc.registerCallback( PUBLISH, boost::bind(&StringPublisher::publish, &sp, _1) );
  sc.registerCallback( RECORD, boost::bind(&Recorder::record<std_msgs::String>, &rec, _1) );

  Float32Converter fc;
  fc.registerCallback( PUBLISH, boost::bind(&Float32Publisher::publish, &fp, _1) );
  fc.registerCallback( RECORD, boost::bind(&Recorder::record<std_msgs::Float32>, &rec, _1) );

  Int32Converter ic;
  ic.registerCallback( PUBLISH, boost::bind(&Int32Publisher::publish, &ip, _1, _2) );
  boost::bind(&Recorder::record<std_msgs::Int32>, &rec, _1, _2);
  //ic.registerCallback( RECORD, boost::bind(&Recorder::record<std_msgs::Int32>, &rec, _1, _2) );

  std::vector< boost::function<void( std::vector<MessageAction> )> > converters;
  converters.push_back( sc );
  converters.push_back( fc );
  converters.push_back( ic );

  bool publish_enabled = true;
  bool record_enabled = false;

  std::vector< MessageAction > actions;
  if (publish_enabled)
  actions.push_back( PUBLISH );
  if ( record_enabled)
  actions.push_back( RECORD );


  for( size_t i=0; i< converters.size(); ++i)
  {
    converters[i]( actions );
  }

  return 0;
}

