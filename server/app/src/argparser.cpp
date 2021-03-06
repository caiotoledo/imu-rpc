#include <string>
#include <sstream>
#include <limits>

#include <argp.h>

#include "argparser.hpp"
#include "argvalidator.hpp"

#define STRINGIFY(x) #x
#define TOSTRING(x)  STRINGIFY(x)

#define STRING_VERSION (TOSTRING(VERSION_MAJOR) "." TOSTRING(VERSION_MINOR) "." TOSTRING(VERSION_PATCH))

static error_t parse_opt (int key, char *arg, struct argp_state *state);

/* Default value for Alpha Constant */
constexpr double DEFAULT_ALPHA = 0.7143;

/* Software Version */
const char *argp_program_version = STRING_VERSION;
/* Contact e-mail */
const char *argp_program_bug_address = "<caioviniciusdetoledo@gmail.com>";
/* Documentation */
static char doc[] = "IMU Daemon RPC";
/* Arguments */
static char args_doc[] = "";

/* Option Arguments */
static struct argp_option options[] = {
  {"daemon", 'd', 0,       0, "Execute as Daemon",                                                                              0},
  {"rate",   'r', "RATE",  0, "Configure Sample Rate in [ms],\n\tvalid values: {10,20,50,100,200,500}\n\tdefault value: 50",    1},
  {"gyro",   'g', "GYRO",  0, "Configure Gyroscope Scale in [°/s],\n\tvalid values: {250,500,1000,2000}\n\tdefault value: 250", 1},
  {"accel",  'a', "ACCEL", 0, "Configure Accelerometer Scale in [G],\n\tvalid values: {2,4,8,16}\n\tdefault value: 2",          1},
  {"alpha",  'c', "ALPHA", 0, "Alpha constant for Complementary Filter, valid values:\n\t{0~1}\n\tdefault value: 0.7143",       1},
  { 0 }
};

/* Arguments Configuration */
static struct argp argp_configuration = { options, parse_opt, args_doc, doc };

static error_t parse_opt (int key, char *arg, struct argp_state *state)
{
  ArgParser::arguments *arguments = (ArgParser::arguments *)state->input;
  switch (key)
    {
      case 'a':
        {
          std::stringstream strValue;
          strValue << arg;
          strValue >> arguments->accel_scale;
        }
      break;
      case 'c':
        {
          std::stringstream strValue;
          strValue << arg;
          strValue >> arguments->const_alpha;
        }
      break;
      case 'd':
        arguments->daemon = true;
      break;
      case 'g':
        {
          std::stringstream strValue;
          strValue << arg;
          strValue >> arguments->gyro_scale;
        }
      break;
      case 'r':
        {
          std::stringstream strValue;
          strValue << arg;
          strValue >> arguments->sample_rate;
        }
      break;

      case ARGP_KEY_INIT:
        arguments->daemon = false;
        arguments->accel_scale = 2;
        arguments->gyro_scale = 250;
        arguments->sample_rate = 50;
        arguments->const_alpha = DEFAULT_ALPHA;
      break;

      case ARGP_KEY_FINI:
        {
          if (!ArgValidator::IsValidArgs(*arguments))
          {
            argp_error(state, "Invalid Parameters Value");
          }
        }
      break;

      case ARGP_KEY_ARG:
        {
          argp_usage (state);
        }
      break;

      default:
        return ARGP_ERR_UNKNOWN;
    }
  return 0;
}

int ArgParser::iProcessArgs(int argc, char const **argv, arguments &args) {
  return (int)argp_parse(&argp_configuration, argc, (char **)argv, 0, 0, &args);
}
