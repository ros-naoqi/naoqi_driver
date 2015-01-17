#include <iostream>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include "alrosbridge.hpp"

namespace alros
{

void Bridge::registerPublisher( const publisher::Publisher& pub )
{
  std::cout << "registered publisher:\t" << pub.name() << std::endl;
  all_publisher_.push_back( pub );
}

void Bridge::publish( )
{
  foreach( publisher::Publisher& p, all_publisher_ )
  {
    p.publish();
  }
}


} //alros
