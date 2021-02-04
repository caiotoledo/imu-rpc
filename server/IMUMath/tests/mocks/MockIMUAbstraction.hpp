#pragma once

#include <functional>

#include <gmock/gmock.h>

#include <IIMUAbstraction.hpp>

namespace IMUAbstraction
{

  class MockIMUAbstraction : public IIMUAbstraction {
  public:
    MOCK_METHOD0(Init, eIMUAbstractionError(void));

    /**
     * GMock working with rvalue (https://stackoverflow.com/questions/12088537/workaround-for-gmock-to-support-rvalue-reference)
     */
    void AddUpdateDataCallback(std::function<void()> &&cb){return AddUpdateDataCallback_rv(cb);}
    MOCK_METHOD1(AddUpdateDataCallback_rv, void(std::function<void()> cb));

    MOCK_METHOD1(SetSampleFrequency, eIMUAbstractionError(eSampleFreq));
    MOCK_METHOD0(GetSampleFrequency, int(void));
    MOCK_METHOD2(GetRawAccel, eIMUAbstractionError(DBusTypes::eAxis, double &));
    MOCK_METHOD1(SetAccelScale, eIMUAbstractionError(eAccelScale));
    MOCK_METHOD2(GetRawGyro, eIMUAbstractionError(DBusTypes::eAxis, double &));
    MOCK_METHOD1(SetGyroScale, eIMUAbstractionError(eGyroScale));

    MOCK_METHOD0(DeInit, void(void));
  };

} // namespace IMUAbstraction
