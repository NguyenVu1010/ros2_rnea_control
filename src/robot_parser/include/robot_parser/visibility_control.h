#ifndef ROBOT_PARSER__VISIBILITY_CONTROL_H_
#define ROBOT_PARSER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROBOT_PARSER_EXPORT __attribute__ ((dllexport))
    #define ROBOT_PARSER_IMPORT __attribute__ ((dllimport))
  #else
    #define ROBOT_PARSER_EXPORT __declspec(dllexport)
    #define ROBOT_PARSER_IMPORT __declspec(dllimport)
  #endif
  #ifdef ROBOT_PARSER_BUILDING_LIBRARY
    #define ROBOT_PARSER_PUBLIC ROBOT_PARSER_EXPORT
  #else
    #define ROBOT_PARSER_PUBLIC ROBOT_PARSER_IMPORT
  #endif
  #define ROBOT_PARSER_PUBLIC_TYPE ROBOT_PARSER_PUBLIC
  #define ROBOT_PARSER_LOCAL
#else
  #define ROBOT_PARSER_EXPORT __attribute__ ((visibility("default")))
  #define ROBOT_PARSER_IMPORT
  #if __GNUC__ >= 4
    #define ROBOT_PARSER_PUBLIC __attribute__ ((visibility("default")))
    #define ROBOT_PARSER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ROBOT_PARSER_PUBLIC
    #define ROBOT_PARSER_LOCAL
  #endif
  #define ROBOT_PARSER_PUBLIC_TYPE
#endif

#endif  // ROBOT_PARSER__VISIBILITY_CONTROL_H_
