#ifndef ROBOT_DYNAMICS__VISIBILITY_CONTROL_H_
#define ROBOT_DYNAMICS__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROBOT_DYNAMICS_EXPORT __attribute__ ((dllexport))
    #define ROBOT_DYNAMICS_IMPORT __attribute__ ((dllimport))
  #else
    #define ROBOT_DYNAMICS_EXPORT __declspec(dllexport)
    #define ROBOT_DYNAMICS_IMPORT __declspec(dllimport)
  #endif
  #ifdef ROBOT_DYNAMICS_BUILDING_LIBRARY
    #define ROBOT_DYNAMICS_PUBLIC ROBOT_DYNAMICS_EXPORT
  #else
    #define ROBOT_DYNAMICS_PUBLIC ROBOT_DYNAMICS_IMPORT
  #endif
  #define ROBOT_DYNAMICS_PUBLIC_TYPE ROBOT_DYNAMICS_PUBLIC
  #define ROBOT_DYNAMICS_LOCAL
#else
  #define ROBOT_DYNAMICS_EXPORT __attribute__ ((visibility("default")))
  #define ROBOT_DYNAMICS_IMPORT
  #if __GNUC__ >= 4
    #define ROBOT_DYNAMICS_PUBLIC __attribute__ ((visibility("default")))
    #define ROBOT_DYNAMICS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ROBOT_DYNAMICS_PUBLIC
    #define ROBOT_DYNAMICS_LOCAL
  #endif
  #define ROBOT_DYNAMICS_PUBLIC_TYPE
#endif

#endif  // ROBOT_DYNAMICS__VISIBILITY_CONTROL_H_
