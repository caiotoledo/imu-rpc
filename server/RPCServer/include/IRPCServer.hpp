#pragma once

#include <functional>

namespace RPCServer
{

  enum class eRPCError {
    eRET_OK,
    eRET_ERROR
  };

  class IRPCServer
  {
  public:
    IRPCServer() = default;

    /**
     * @brief Initialize Request Handler
     *
     * @return eRPCError Returns #eRET_OK when successful, ref #eRPCError
     */
    virtual eRPCError Init(void) = 0;

    /**
     * @brief Send a signal notification about IMU data updated
     *
     * @return eRPCError Returns #eRET_OK when successful, ref #eRPCError
     */
    virtual eRPCError NotifyDataUpdate(void) = 0;

    /**
     * @brief Register GetRawAccel Callback
     *
     * @param cb Callback to be called
     * @return eRPCError Returns #eRET_OK when successful, ref #eRPCError
     */
    virtual eRPCError setGetRawAccelCallback(std::function<double(int)> &&cb) = 0;

    /**
     * @brief Register GetRawGyro Callback
     *
     * @param cb Callback to be called
     * @return eRPCError Returns #eRET_OK when successful, ref #eRPCError
     */
    virtual eRPCError setGetRawGyroCallback(std::function<double(int)> &&cb) = 0;

    /**
     * @brief Register GetEulerAngles Callback
     *
     * @param cb Callback to be called
     * @return eRPCError Returns #eRET_OK when successful, ref #eRPCError
     */
    virtual eRPCError setGetEulerAngleCallback(std::function<double(int, int)> &&cb) = 0;

    /**
     * @brief Register GetComplFilterAngle Callback
     *
     * @param cb Callback to be called
     * @return eRPCError Returns #eRET_OK when successful, ref #eRPCError
     */
    virtual eRPCError setGetComplFilterAngleCallback(std::function<double(int, int)> &&cb) = 0;

    /**
     * @brief DeInitialize Request Handler
     */
    virtual void DeInit(void) = 0;

    virtual ~IRPCServer() = default;
  };

} // namespace RPCServer
