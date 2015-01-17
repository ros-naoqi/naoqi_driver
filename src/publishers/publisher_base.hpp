#ifndef BASE_PUBLISHER_HPP
#define BASE_PUBLISHER_HPP

namespace alros
{
namespace publisher
{

class BasePublisher
{

public:
  BasePublisher( const std::string& name, const std::string& topic ):
    name_( name ),
    topic_( topic )
  {}

  inline std::string name() const
  {
    return name_;
  }

  inline std::string topic() const
  {
    return topic_;
  }

private:
  std::string name_, topic_;

}; // class

} //publisher
} // alros

#endif
