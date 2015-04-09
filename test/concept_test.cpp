#include <alrosbridge/test/old_concept.hpp>
#include <alrosbridge/test/new_concept.hpp>

#include <vector>

class Object{
public:
  Object(std::string message){
    _msg = message;
  }

  std::string method() const
  {
    return _msg;
  }

  void set_message(std::string message){
    _msg = message;
  }

private:
  std::string _msg;
};

class FakeBridge
{
public:
  void registerObjectOld(alros::concept::Old object)
  {
    _olds.push_back(object);
  }

  void registerObjectNew(alros::concept::New object)
  {
    _news.push_back(object);
  }

  void callAll()
  {
    std::cout << "Call objects in old concept form" << std::endl;
    for(int i=0; i<_olds.size(); i++)
    {
      std::cout << "\t" << _olds[i].method() << std::endl;
    }
    std::cout << "Call objects in new concept form" << std::endl;
    for(int i=0; i<_news.size(); i++)
    {
      std::cout << "\t" << _news[i].method() << std::endl;
    }
    std::cout << std::endl;
  }

private:
  std::vector<alros::concept::Old> _olds;
  std::vector<alros::concept::New> _news;
};

int main()
{
  FakeBridge _fb;

  // Register objects
  boost::shared_ptr<Object> original_object = boost::make_shared<Object>("Function A");
  _fb.registerObjectOld(*original_object);
  _fb.registerObjectNew(original_object);


  // Call all
  _fb.callAll();

  // Modify the object
  original_object->set_message("Two");

  // Call all
  _fb.callAll();

  return 0;
}
