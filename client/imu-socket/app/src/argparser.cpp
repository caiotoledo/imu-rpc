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
static char doc[] = "IMU Socket";
/* Arguments */
static char args_doc[] = "";

/* Option Arguments */
static struct argp_option options[] = {
  {"daemon", 'd', 0,      0, "Execute as Daemon", 0},
  {"udp",    'u', 0,      0, "Use UDP Socket",    0},
  {"port",   'p', "PORT", 0, "Socket Port",       0},
  { 0 }
};

/* Arguments Configuration */
static struct argp argp_configuration = { options, parse_opt, args_doc, doc };

static error_t parse_opt (int key, char *arg, struct argp_state *state)
{
  ArgParser::arguments *arguments = (ArgParser::arguments *)state->input;
  switch (key)
    {
      case 'd':
        arguments->daemon = true;
      break;
      case 'u':
        arguments->udp = true;
      break;
      case 'p':
        {
          std::stringstream strValue;
          strValue << arg;
          strValue >> arguments->port;
        }
      break;

      case ARGP_KEY_INIT:
        arguments->daemon = false;
        arguments->udp = false;
        arguments->port = 1111;
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
