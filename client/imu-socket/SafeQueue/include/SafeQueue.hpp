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

    /**
     * @brief Push new data to queue
     *
     * @param data Data variable
     */
    void enqueue(T data)
    {
      std::lock_guard<std::mutex> lck(mData);
      qData.push(data);
      cvData.notify_one();
    }

    /**
     * @brief Pop data from queue
     *
     * @param data Variable to store data
     * @param timeout Wait time in case of queue empty in ms (If timeout < 0, then waits forever)
     * @return true Successful retrieve data
     * @return false Timeout
     */
    bool dequeue(T &data, int timeout)
    {
      auto ret = true;

      std::unique_lock<std::mutex> lock(mData);

      /* Wait for data available in queue */
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
        /* Retrieve data from queue */
        data = qData.front();
        qData.pop();
      }

      return ret;
    }

    ~SafeQueue() = default;
  };

} // namespace Queue
