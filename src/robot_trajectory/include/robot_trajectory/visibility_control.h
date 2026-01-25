#ifndef ROBOT_TRAJECTORY__VISIBILITY_CONTROL_H_
#define ROBOT_TRAJECTORY__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROBOT_TRAJECTORY_EXPORT __attribute__ ((dllexport))
    #define ROBOT_TRAJECTORY_IMPORT __attribute__ ((dllimport))
  #else
    #define ROBOT_TRAJECTORY_EXPORT __declspec(dllexport)
    #define ROBOT_TRAJECTORY_IMPORT __declspec(dllimport)
  #endif
  #ifdef ROBOT_TRAJECTORY_BUILDING_LIBRARY
    #define ROBOT_TRAJECTORY_PUBLIC ROBOT_TRAJECTORY_EXPORT
  #else
    #define ROBOT_TRAJECTORY_PUBLIC ROBOT_TRAJECTORY_IMPORT
  #endif
  #define ROBOT_TRAJECTORY_PUBLIC_TYPE ROBOT_TRAJECTORY_PUBLIC
  #define ROBOT_TRAJECTORY_LOCAL
#else
  #define ROBOT_TRAJECTORY_EXPORT __attribute__ ((visibility("default")))
  #define ROBOT_TRAJECTORY_IMPORT
  #if __GNUC__ >= 4
    #define ROBOT_TRAJECTORY_PUBLIC __attribute__ ((visibility("default")))
    #define ROBOT_TRAJECTORY_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ROBOT_TRAJECTORY_PUBLIC
    #define ROBOT_TRAJECTORY_LOCAL
  #endif
  #define ROBOT_TRAJECTORY_PUBLIC_TYPE
#endif

#endif  // ROBOT_TRAJECTORY__VISIBILITY_CONTROL_H_
