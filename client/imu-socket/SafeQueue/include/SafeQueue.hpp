#pragma once

#include <condition_variable>
#include <chrono>
#include <queue>
#include <mutex>

namespace Queue
{

  template <typename T>
  class SafeQueue
  {
  private:
    std::queue<T> qData;
    std::mutex mData;
    std::condition_variable cvData;

  public:
    SafeQueue() :
      qData(),
      mData(),
      cvData()
    {}

    void enqueue(T data)
    {
      std::lock_guard<std::mutex> lck(mData);
      qData.push(data);
      cvData.notify_one();
    }

    bool dequeue(T &data, int timeout)
    {
      auto ret = true;

      std::unique_lock<std::mutex> lock(mData);
      if (timeout >= 0)
      {
        ret = cvData.wait_for(lock, std::chrono::milliseconds(timeout), [&]{ return !qData.empty(); } );
      }
      else
      {
        cvData.wait(lock, [&]{ return !qData.empty(); });
      }

      if (ret)
      {
        data = qData.front();
        qData.pop();
      }

      return ret;
    }

    ~SafeQueue() = default;
  };

} // namespace Queue
