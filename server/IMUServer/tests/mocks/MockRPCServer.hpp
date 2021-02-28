#pragma once

#include <functional>

#include <gmock/gmock.h>

#include <IRPCServer.hpp>

namespace RPCServer
{

  class MockRPCServer : public IRPCServer {
  public:
    MOCK_METHOD0(Init, eRPCError(void));
    MOCK_METHOD0(NotifyDataUpdate, eRPCError(void));

    /**
     * GMock working with rvalue (https://stackoverflow.com/questions/12088537/workaround-for-gmock-to-support-rvalue-reference)
     */
    eRPCError setGetRawAccelCallback(std::function<double(int)> &&cb){return setGetRawAccelCallback_rv(cb);}
    MOCK_METHOD1(setGetRawAccelCallback_rv, eRPCError(std::function<double(int)> cb));
    eRPCError setGetRawGyroCallback(std::function<double(int)> &&cb){return setGetRawGyroCallback_rv(cb);}
    MOCK_METHOD1(setGetRawGyroCallback_rv, eRPCError(std::function<double(int)> cb));
    eRPCError setGetEulerAngleCallback(std::function<double(int, int)> &&cb){return setGetEulerAngleCallback_rv(cb);}
    MOCK_METHOD1(setGetEulerAngleCallback_rv, eRPCError(std::function<double(int, int)> cb));
    eRPCError setGetComplFilterAngleCallback(std::function<double(int, int)> &&cb){return setGetComplFilterAngleCallback_rv(cb);}
    MOCK_METHOD1(setGetComplFilterAngleCallback_rv, eRPCError(std::function<double(int, int)> cb));

    MOCK_METHOD0(DeInit, void(void));
  };

} // namespace IMUMath
