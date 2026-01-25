#ifndef ROBOT_COMMON__VISIBILITY_CONTROL_H_
#define ROBOT_COMMON__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROBOT_COMMON_EXPORT __attribute__ ((dllexport))
    #define ROBOT_COMMON_IMPORT __attribute__ ((dllimport))
  #else
    #define ROBOT_COMMON_EXPORT __declspec(dllexport)
    #define ROBOT_COMMON_IMPORT __declspec(dllimport)
  #endif
  #ifdef ROBOT_COMMON_BUILDING_LIBRARY
    #define ROBOT_COMMON_PUBLIC ROBOT_COMMON_EXPORT
  #else
    #define ROBOT_COMMON_PUBLIC ROBOT_COMMON_IMPORT
  #endif
  #define ROBOT_COMMON_PUBLIC_TYPE ROBOT_COMMON_PUBLIC
  #define ROBOT_COMMON_LOCAL
#else
  #define ROBOT_COMMON_EXPORT __attribute__ ((visibility("default")))
  #define ROBOT_COMMON_IMPORT
  #if __GNUC__ >= 4
    #define ROBOT_COMMON_PUBLIC __attribute__ ((visibility("default")))
    #define ROBOT_COMMON_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ROBOT_COMMON_PUBLIC
    #define ROBOT_COMMON_LOCAL
  #endif
  #define ROBOT_COMMON_PUBLIC_TYPE
#endif

#endif  // ROBOT_COMMON__VISIBILITY_CONTROL_H_
