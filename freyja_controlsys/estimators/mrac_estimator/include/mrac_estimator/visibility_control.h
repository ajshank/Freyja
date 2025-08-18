#ifndef MRAC_ESTIMATOR__VISIBILITY_CONTROL_H_
#define MRAC_ESTIMATOR__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define MRAC_ESTIMATOR_EXPORT __attribute__ ((dllexport))
    #define MRAC_ESTIMATOR_IMPORT __attribute__ ((dllimport))
  #else
    #define MRAC_ESTIMATOR_EXPORT __declspec(dllexport)
    #define MRAC_ESTIMATOR_IMPORT __declspec(dllimport)
  #endif
  #ifdef MRAC_ESTIMATOR_BUILDING_LIBRARY
    #define MRAC_ESTIMATOR_PUBLIC MRAC_ESTIMATOR_EXPORT
  #else
    #define MRAC_ESTIMATOR_PUBLIC MRAC_ESTIMATOR_IMPORT
  #endif
  #define MRAC_ESTIMATOR_PUBLIC_TYPE MRAC_ESTIMATOR_PUBLIC
  #define MRAC_ESTIMATOR_LOCAL
#else
  #define MRAC_ESTIMATOR_EXPORT __attribute__ ((visibility("default")))
  #define MRAC_ESTIMATOR_IMPORT
  #if __GNUC__ >= 4
    #define MRAC_ESTIMATOR_PUBLIC __attribute__ ((visibility("default")))
    #define MRAC_ESTIMATOR_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define MRAC_ESTIMATOR_PUBLIC
    #define MRAC_ESTIMATOR_LOCAL
  #endif
  #define MRAC_ESTIMATOR_PUBLIC_TYPE
#endif

#endif  // MRAC_ESTIMATOR__VISIBILITY_CONTROL_H_
