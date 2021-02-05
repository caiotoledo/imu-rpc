#pragma once

namespace ArgParser
{
  struct arguments {
    bool daemon;
    double const_alpha;
    int accel_scale;
    int gyro_scale;
    int sample_rate;
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


