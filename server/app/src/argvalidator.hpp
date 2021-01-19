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

} // namespace ArgValidator
