#include <memory>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <MockIMUAbstraction.hpp>

#include <IMUMathImpl.hpp>

using ::testing::_;
using ::testing::Return;

TEST(imumathimpl, imumath_init)
{
  auto imuMock = std::make_shared<IMUAbstraction::MockIMUAbstraction>();

  std::shared_ptr<IMUMath::IIMUMath> imuMath;
  imuMath = std::make_shared<IMUMath::IMUMathImpl>(imuMock);

  EXPECT_CALL(*imuMock, Init())
    .Times(1)
    .WillRepeatedly(Return(IMUAbstraction::eIMUAbstractionError::eRET_OK));

  auto ret = imuMath->Init();

  EXPECT_EQ(ret, IMUMath::eIMUMathError::eRET_OK);
}
