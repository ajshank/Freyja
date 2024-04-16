#ifndef FREYJA_UTILS__VISIBILITY_CONTROL_H_
#define FREYJA_UTILS__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define FREYJA_UTILS_EXPORT __attribute__ ((dllexport))
    #define FREYJA_UTILS_IMPORT __attribute__ ((dllimport))
  #else
    #define FREYJA_UTILS_EXPORT __declspec(dllexport)
    #define FREYJA_UTILS_IMPORT __declspec(dllimport)
  #endif
  #ifdef FREYJA_UTILS_BUILDING_LIBRARY
    #define FREYJA_UTILS_PUBLIC FREYJA_UTILS_EXPORT
  #else
    #define FREYJA_UTILS_PUBLIC FREYJA_UTILS_IMPORT
  #endif
  #define FREYJA_UTILS_PUBLIC_TYPE FREYJA_UTILS_PUBLIC
  #define FREYJA_UTILS_LOCAL
#else
  #define FREYJA_UTILS_EXPORT __attribute__ ((visibility("default")))
  #define FREYJA_UTILS_IMPORT
  #if __GNUC__ >= 4
    #define FREYJA_UTILS_PUBLIC __attribute__ ((visibility("default")))
    #define FREYJA_UTILS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define FREYJA_UTILS_PUBLIC
    #define FREYJA_UTILS_LOCAL
  #endif
  #define FREYJA_UTILS_PUBLIC_TYPE
#endif

#endif  // FREYJA_UTILS__VISIBILITY_CONTROL_H_
