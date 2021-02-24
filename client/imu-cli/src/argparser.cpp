#include <string>
#include <sstream>

#include <argp.h>

#include "argparser.hpp"

#define STRINGIFY(x) #x
#define TOSTRING(x)  STRINGIFY(x)

#define STRING_VERSION (TOSTRING(VERSION_MAJOR) "." TOSTRING(VERSION_MINOR) "." TOSTRING(VERSION_PATCH))

static error_t parse_opt (int key, char *arg, struct argp_state *state);

/* Software Version */
const char *argp_program_version = STRING_VERSION;
/* Contact e-mail */
const char *argp_program_bug_address = "<caioviniciusdetoledo@gmail.com>";
/* Documentation */
static char doc[] = "IMU Command Line Interface";
/* Arguments */
static char args_doc[] = "";

/* Option Arguments */
static struct argp_option options[] = {
  {"accel",       'a', 0,      0, "Sample Raw Accelerometer Values [mG]",         0},
  {"gyro",        'g', 0,      0, "Sample Raw Gyroscope Values [°/s]",            0},
  {"euler",       'e', 0,      0, "Sample Euler Angle [°]",                       0},
  {"compl_angle", 'c', 0,      0, "Sample Complementary Filter Euler Angle [°]",  0},
  {"timeout",     't', "TOUT", 0, "Define application runtime in seconds",        1},
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
        arguments->accel = true;
      break;
      case 'c':
        arguments->compl_filter_angle = true;
      break;
      case 'g':
        arguments->gyro = true;
      break;
      case 'e':
        arguments->euler = true;
      break;
      case 't':
        {
          std::stringstream strValue;
          strValue << arg;
          strValue >> arguments->timeout;
        }
      break;

      case ARGP_KEY_INIT:
        arguments->accel = false;
        arguments->gyro = false;
        arguments->euler = false;
        arguments->compl_filter_angle = false;
        arguments->timeout = -1;
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
