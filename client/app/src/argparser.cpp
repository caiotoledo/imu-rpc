#include <string>
#include <sstream>

#include <argp.h>

#include "argparser.hpp"

static error_t parse_opt (int key, char *arg, struct argp_state *state);

/* Software Version */
const char *argp_program_version = "0.0.0";
/* Contact e-mail */
const char *argp_program_bug_address = "<caioviniciusdetoledo@gmail.com>";
/* Documentation */
static char doc[] = "";
/* Arguments */
static char args_doc[] = "";

/* Option Arguments */
static struct argp_option options[] = {
  {"timeout", 't', "TOUT", 0, "Define application runtime",           0},
  {"accel",   'a', 0,      0, "Sample Raw Accelerometer Values [mG]", 0},
  {"gyro",    'g', 0,      0, "Sample Raw Gyroscope Values [°/s]",    0},
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
      case 'g':
        arguments->gyro = true;
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
