/**
 * test for having a cyclic loop and connecting those items with various callbacks
 */

#include <iostream>
#include <string>
#include <vector>
#include <boost/function.hpp>
#include <boost/bind.hpp>

/**
 * Emulate different message types here
 */
struct StringMsg
{
  std::string data;
};
struct FloatMsg
{
  float data;
};
struct IntMsg
{
  int data;
};

/**
 * Emulate different publishers here
 */
struct StringPublisher
{
  void publish( const StringMsg& m )
  {
    std::cout << "String Publisher publishes " << m.data << std::endl;
  }
};
struct FloatPublisher
{
  void publish( const FloatMsg& m )
  {
    std::cout << "Float Publisher publishes " << m.data << std::endl;
  }
};
struct IntPublisher
{
  void publish( const IntMsg& m )
  {
    std::cout << "Int Publisher publishes " << m.data << std::endl;
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

  void record( const IntMsg& m )
  {
    std::cout << "Recorder writes directly a cool InterMsg " << std::endl;
  }
};

/**
 * Emulate the translation unit
 */

struct StringConverter //: public BaseConverter
{
  typedef boost::function<void(StringMsg)> Callback;

  Callback callback_function_publish;
  Callback callback_function_record;

  StringConverter(Callback publish_callback, Callback record_callback)
    : callback_function_publish(publish_callback)
    , callback_function_record(record_callback)
  {}

  // should get a callback to the recorder and the 
  void operator()( bool publish_enabled, bool record_enabled ) const
  {
    StringMsg m;
    m.data = "Hello String";

    if ( publish_enabled )
	    callback_function_publish(m);
    if ( record_enabled )
	    callback_function_record(m);
  }
};

struct FloatConverter
{
  typedef boost::function<void(FloatMsg)> Callback;

  Callback callback_function_publish;
  Callback callback_function_record;

  FloatConverter(Callback publish_callback, Callback record_callback)
    : callback_function_publish(publish_callback)
    , callback_function_record(record_callback)
  {}


  void operator()( bool publish_enabled, bool record_enabled ) const
  {
    FloatMsg float_msg;
    float_msg.data = 1.5f;
    if ( publish_enabled )
	    callback_function_publish(float_msg);
    if ( record_enabled )
	    callback_function_record(float_msg);

  }
};

template<class MessageType>
struct CallbackCluster
{
  typedef boost::function<void(MessageType)> Callback;

  Callback callback_function_publish;
  Callback callback_function_record;

 CallbackCluster(Callback publish_callback, Callback record_callback)
    : callback_function_publish(publish_callback)
    , callback_function_record(record_callback)
  {}


  void operator()( const MessageType& msg, bool publish_enabled, bool record_enabled ) const
  {
    if ( publish_enabled )
	    callback_function_publish(msg);
    if ( record_enabled )
	    callback_function_record(msg);
  }

};

struct IntConverter
{
  CallbackCluster<IntMsg> callbacks;

  IntConverter(CallbackCluster::Callback publish_callback, CallbackCluster::Callback record_callback)
    : callbacks(publish_callback, record_callback)
  {}

  void operator()( bool publish_enabled, bool record_enabled ) const
  {
    IntMsg int_msg;
    int_msg.data = 1;
    callbacks(msg, publish_enabled, record_enabled );
  }

};

void lol(int i, std::string k);

int main()
{
  StringPublisher sp;
  FloatPublisher fp;
  IntPublisher ip;

  Recorder rec;

  std::vector< boost::function<void( bool, bool )> > converters;
  converters.push_back( StringConverter( boost::bind(&StringPublisher::publish, &sp, _1), boost::bind(&Recorder::record<StringMsg>, &rec, _1) ) );
  converters.push_back( FloatConverter( boost::bind(&FloatPublisher::publish, &fp, _1), boost::bind(&Recorder::record<FloatMsg>, &rec, _1) ) );
  converters.push_back( IntConverter( boost::bind(&IntPublisher::publish, &ip, _1), boost::bind(&Recorder::record<IntMsg>, &rec, _1) ) );

  bool publish_enabled = true;
  bool record_enabled = false;

  for( size_t i=0; i< converters.size(); ++i)
  {
	converters[i]( publish_enabled, record_enabled );
  }
  //Recorder rec;
  //rec.record(float_msg);
  //rec.record(string_msg);
  //rec.record(int_msg);

  return 0;
}

