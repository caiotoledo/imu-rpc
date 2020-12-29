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

  static std::shared_ptr<T> GetInstance()
  {
    static std::shared_ptr<T> instance = NULL;
    /* Check if the instance wasn't initialize yet */
    if (!instance)
    {
      instance = std::make_shared<T>();
    }
    return instance;
  }
};

} // namespace singleton
