#ifndef IMAGE_BASE_PUBLISHER_HPP
#define IMAGE_BASE_PUBLISHER_HPP

#include <iostream>

namespace alros
{
namespace publisher
{

// CRTP
template<class T>
class ImageBasePublisher
{

public:
  ImageBasePublisher( const std::string& name, const std::string& topic, float frequency ):
    name_( name ),
    topic_( topic ),
    frequency_( frequency ),
    is_initialized_( false )
  {}

  virtual ~ImageBasePublisher() {};

  inline std::string name() const
  {
    return name_;
  }

  inline std::string topic() const
  {
    return topic_;
  }

  inline float frequency() const
  {
    return frequency_;
  }

  inline bool isInitialized() const
  {
    return is_initialized_;
  }

  inline bool isSubscribed() const
  {
    if (is_initialized_ == false) return false;
    return pub_.getNumSubscribers() > 0;
  }

protected:
  std::string name_, topic_;

  //image_transport::ImageTransport it_;
  image_transport::Publisher pub_;
  bool is_initialized_;

  /** Frequency at which the publisher will publish */
  float frequency_;
}; // class

} //publisher
} // alros

#endif
