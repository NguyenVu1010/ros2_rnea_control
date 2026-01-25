#ifndef RNEA_CONTROLLER__VISIBILITY_CONTROL_H_
#define RNEA_CONTROLLER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define RNEA_CONTROLLER_EXPORT __attribute__ ((dllexport))
    #define RNEA_CONTROLLER_IMPORT __attribute__ ((dllimport))
  #else
    #define RNEA_CONTROLLER_EXPORT __declspec(dllexport)
    #define RNEA_CONTROLLER_IMPORT __declspec(dllimport)
  #endif
  #ifdef RNEA_CONTROLLER_BUILDING_LIBRARY
    #define RNEA_CONTROLLER_PUBLIC RNEA_CONTROLLER_EXPORT
  #else
    #define RNEA_CONTROLLER_PUBLIC RNEA_CONTROLLER_IMPORT
  #endif
  #define RNEA_CONTROLLER_PUBLIC_TYPE RNEA_CONTROLLER_PUBLIC
  #define RNEA_CONTROLLER_LOCAL
#else
  #define RNEA_CONTROLLER_EXPORT __attribute__ ((visibility("default")))
  #define RNEA_CONTROLLER_IMPORT
  #if __GNUC__ >= 4
    #define RNEA_CONTROLLER_PUBLIC __attribute__ ((visibility("default")))
    #define RNEA_CONTROLLER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define RNEA_CONTROLLER_PUBLIC
    #define RNEA_CONTROLLER_LOCAL
  #endif
  #define RNEA_CONTROLLER_PUBLIC_TYPE
#endif

#endif  // RNEA_CONTROLLER__VISIBILITY_CONTROL_H_
