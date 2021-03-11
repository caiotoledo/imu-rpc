#pragma once

#include <chrono>

namespace Timer
{

  class Timer
  {
  public:
    Timer() = default;

    /**
     * @brief Get the time since epoch in seconds
     *
     * @return double Seconds since epoch
     */
    double GetSeconds(void)
    {
      auto now_since_epoch = std::chrono::high_resolution_clock::now().time_since_epoch();
      auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now_since_epoch);

      return ((double)duration_ms.count())/1000.0;
    }

    ~Timer() = default;
  };

} // namespace Timer

