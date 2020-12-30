#include <string>
#include <sstream>
#include <limits>

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
  {"daemon", 'd', 0, 0, "Execute as Daemon", 0},
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

      case ARGP_KEY_INIT:
        arguments->daemon = false;
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
