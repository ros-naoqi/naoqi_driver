#include <iostream>

// in main.cpp
#include "alrosbridge.hpp"
#include "publishers/publisher_string.hpp"
#include "publishers/publisher_int.hpp"


int main(int argc, char *argv[])
{

 std::cout << "application started " << std::endl;

  alros::publisher::StringPublisher string_pub( "string_publisher", "string_topic" );
  alros::publisher::IntPublisher int_pub( "int_publisher", "int_topic" );

  string_pub.publish();
  int_pub.publish();


  alros::Bridge bridge;
  bridge.registerPublisher( string_pub );
  bridge.registerPublisher( int_pub );

  bridge.publish();

  return 0;
}
