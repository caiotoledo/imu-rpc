#pragma once

#include <IIMUAbstraction.hpp>

#include "argparser.hpp"

namespace ArgValidator
{

  /**
   * @brief Check if arguments is valid
   *
   * @param args Object arguments
   * @return true Valid args
   * @return false Invalid arg value
   */
  bool IsValidArgs(const ArgParser::arguments &args);

  /**
   * @brief Convert arguments to IMUAbstraction types
   *
   * @param args Object arguments
   * @param accel Accelerometer Scale
   * @param gyro Gyroscope Scale
   * @param freq Sample Rate
   * @return int Returns 0 in success, -1 otherwise
   */
  int ConvertArgs(const ArgParser::arguments &args,
                  IMUAbstraction::eAccelScale &accel,
                  IMUAbstraction::eGyroScale &gyro,
                  IMUAbstraction::eSampleFreq &freq);

} // namespace ArgValidator
