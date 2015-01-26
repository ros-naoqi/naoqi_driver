#ifndef BENCHMARK_HELPERS_H
#define BENCHMARK_HELPERS

#include <iostream>

namespace helpers
{

static size_t get_buffer_size( const int argc, char** const argv )
{
  size_t buffer_size = 10;

  if ( argc > 2 )
  {
    std::cout << "getting buffer of " << argv[2] << std::endl;
    buffer_size = atoi( argv[2] );
  }
  return buffer_size;
}

static size_t get_frequency( const int argc, char** const argv )
{
  size_t frequency = 10;

  if ( argc > 1 )
  {
    std::cout << "getting frequency of "<< argv[1] << std::endl;
    frequency = atoi( argv[1] );
  }
  return frequency;
}

} // helpers

#endif
