#ifndef PUBLISHER_HPP
#define HPP

#include <string>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>


namespace alros
{
namespace publisher
{

class Publisher
{

public:
  template<typename T>
  Publisher( T pub ):
    pubPtr_( boost::make_shared<PublisherModel<T> >(pub) )
  {};

  void publish()
  {
    pubPtr_->publish();
  }

  std::string name() const
  {
    return pubPtr_->name();
  }

private:

  struct PublisherConcept
  {
    virtual ~PublisherConcept(){};
    virtual std::string name() const = 0;
    virtual void publish() = 0;
    virtual void reset() = 0;
  };

  template<typename T>
  struct PublisherModel : public PublisherConcept
  {
    PublisherModel( const T& other ):
      publisher_( other )
    {}

    std::string name() const
    {
      return publisher_.name();
    }

    void publish()
    {
      publisher_.publish();
    }

    void reset()
    {
      publisher_.reset();
    }

    T publisher_;
  };

  boost::shared_ptr<PublisherConcept> pubPtr_;

}; // class publisher

} //publisher
} //alros

#endif
