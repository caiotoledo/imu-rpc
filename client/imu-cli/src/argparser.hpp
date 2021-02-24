#pragma once

namespace ArgParser
{
  struct arguments {
    int timeout;
    bool accel;
    bool gyro;
    bool euler;
    bool compl_filter_angle;
  };

  /**
   * @brief Parse application arguments
   *
   * @param argc Argument Count
   * @param argv Argument Vector
   * @param args Arguments values to be returned
   * @return int Returns 0 in successful
   */
  int iProcessArgs(int argc, char const **argv, arguments &args);

} // namespace ArgParser


