#pragma once

#include <memory>

namespace singleton
{

template<typename T>
class SingletonInstancer
{
private:
  SingletonInstancer() { }
public:
  ~SingletonInstancer() { }

  template<typename... Args>
  static std::shared_ptr<T> GetInstance(Args... args)
  {
    static std::shared_ptr<T> instance = NULL;
    /* Check if the instance wasn't initialize yet */
    if (!instance)
    {
      instance = std::make_shared<T>(args...);
    }
    return instance;
  }
};

} // namespace singleton
